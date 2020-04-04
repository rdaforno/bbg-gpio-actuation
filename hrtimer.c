/**
 * High resolution timer kernel module example for the BeagleBone Green (tested with kernel 4.14).
 *
 * Creates a periodic timer and toggles a GPIO pin when the timer expires (P9.12 by default).
 * The GPIO pin is actuated using direct memory-mapped IO access (bus / file system independent).
 * The module creates a character device ('/dev/hrtimer') to communicate with a user space program.
 *
 * To e.g. toggle the GPIO with period of 10Hz, write '10' into the device /dev/hrtimer (max. value is 100000).
 *   echo 10 > /dev/hrtimer
 *
 * In order to access the device without root permissions, add a new udev rule to /etc/udev/rules.d/
 *   KERNEL=="hrtimer", OWNER="root", GROUP="dialout", MODE="0666"
 *
 * For the output pin to work, you either need to enable the universal cape and set the pin as output or
 * configure the pin in the device tree overlay accordingly.
 *
 * 2020, rdaforno
 */

// --- INCLUDES ---

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <asm/io.h>


// --- CONFIG / DEFINES ---

#define MODULE_NAME         "[hrtimer] "  // prefix for log printing
#define DEVICE_NAME         "hrtimer"     // name of the device in '/dev/'
#define NUM_ITERATIONS      10            // limits #iterations for the periodic timer
#define DEVICE_BUFFER_SIZE  128           // max. buffer size for character device
#define PIN_NUMBER          60            // pin to toggle, 60 = P9.12


// --- MACROS ---

#define TIMER_NOW_NS()      ktime_get_ns()        // = ktime_to_ns(ktime_get())
#define TIMER_NOW_TS(t)     getnstimeofday(&t);   // returns a timespec struct

#define GPIO0_START_ADDR    0x44E07000            // see am335x RM p.180
#define GPIO1_START_ADDR    0x4804C000
#define GPIO2_START_ADDR    0x481AC000
#define GPIO3_START_ADDR    0x481AE000
#define GPIO_MEM_SIZE       0x2000
#define GPIO_OE_OFS         0x134
#define GPIO_SET_OFS        0x194
#define GPIO_CLR_OFS        0x190
#define PIN_BIT             (1 << (PIN_NUMBER & 31))
#if PIN_NUMBER < 32
  #define GPIO_ADDR         GPIO0_START_ADDR
#elif PIN_NUMBER < 64
  #define GPIO_ADDR         GPIO1_START_ADDR
#elif PIN_NUMBER < 96
  #define GPIO_ADDR         GPIO2_START_ADDR
#else
  #define GPIO_ADDR         GPIO3_START_ADDR
#endif


// --- GLOBAL VARIABLES ---

static struct hrtimer  rt_timer;    // realtime timer
static struct hrtimer  m_timer;     // monotonic timer
static ktime_t         t_period;    // timer period, only used for the monotonic timer
static ktime_t         t_start;
static void            (*rt_timer_cb)(void);  // user defined callback
static void            (*m_timer_cb)(void);   // user defined callback
static bool            auto_restart;

static struct class* hrtimer_dev_class;
static int           hrtimer_dev_major;
static char          hrtimer_dev_data[DEVICE_BUFFER_SIZE];

static volatile unsigned int* gpio_set_addr = NULL;
static volatile unsigned int* gpio_clr_addr = NULL;


// --- FUNCTIONS ---

static void gpio_set(uint32_t pin)
{
  if (gpio_set_addr) {
    *gpio_set_addr = pin;
  }
}

static void gpio_clr(uint32_t pin)
{
  if (gpio_clr_addr) {
    *gpio_clr_addr = pin;
  }
}

static void map_gpio(void)
{
  volatile void*         gpio_addr_mapped;
  volatile unsigned int* gpio_oe_addr;

  gpio_addr_mapped = ioremap(GPIO_ADDR, GPIO_MEM_SIZE);
  gpio_oe_addr     = gpio_addr_mapped + GPIO_OE_OFS;
  gpio_set_addr    = gpio_addr_mapped + GPIO_SET_OFS;
  gpio_clr_addr    = gpio_addr_mapped + GPIO_CLR_OFS;

  if (gpio_addr_mapped == 0) {
    printk(MODULE_NAME "Unable to map GPIO\n");
    return;
  }
  printk(MODULE_NAME "GPIO peripheral address mapped to %p\n", gpio_addr_mapped);
}

// ------------------------------------------

static void my_callback(void)
{
  static bool last_state = false;

  last_state = !last_state;
  if (last_state) {
    gpio_set(PIN_BIT);
  } else {
    gpio_clr(PIN_BIT);
  }
}

// ------------------------------------------

static void timer_reset(struct hrtimer* timer)
{
  // note: t_period is the next timer expiration, relative to the current time
  hrtimer_forward(timer, timer->_softexpires, t_period);      // _softexpires: time that was set for this timer expiration
  //hrtimer_forward(timer, timer->base->get_time(), t_period);  // base->get_time() returns current timer value
}

// timer callback function
static enum hrtimer_restart rt_timer_expired(struct hrtimer *timer)
{
  ktime_t t_now;
  t_now = TIMER_NOW_NS();
  rt_timer_cb();
  printk(MODULE_NAME "timer expired (%lldns)\n", (t_now - t_start));
  t_start = t_now;
  return HRTIMER_NORESTART;
}

// timer callback function
static enum hrtimer_restart m_timer_expired(struct hrtimer *timer)
{
  static uint32_t cnt = 0;
  ktime_t t_now;

  t_now = TIMER_NOW_NS();
  m_timer_cb();
  printk(MODULE_NAME "deviation: %dus\n", (int32_t)(t_now - t_start - t_period) / 1000);
  t_start = t_now;

  if (auto_restart) {
    // set next expiration time
    timer_reset(timer);
    cnt++;
    if (cnt < NUM_ITERATIONS) {
      return HRTIMER_RESTART;
    }
    printk(MODULE_NAME "timer stopped\n");
    cnt = 0;
    return HRTIMER_NORESTART;
  }
  return HRTIMER_NORESTART;
}

// set one shot timer (absolute mode, uses realtime timer)
static void timer_set_abs(struct timespec t_exp, void (*cb)(void))
{
  ktime_t kt;
  if (cb) {
    // make sure the timer is not running anymore
    hrtimer_cancel(&rt_timer);
    rt_timer.function = rt_timer_expired;
    rt_timer_cb = cb;
    kt = ktime_set(t_exp.tv_sec, t_exp.tv_nsec);
    t_start = TIMER_NOW_NS();
    hrtimer_start(&rt_timer, kt, HRTIMER_MODE_ABS);
  }
}

// set one shot timer (relative mode, uses monotonic timer)
static void timer_set_rel(uint32_t period_usec, void (*cb)(void), bool periodic)
{
  uint32_t secs;
  if (cb) {
    // make sure the timer is not running anymore
    hrtimer_cancel(&m_timer);
    m_timer.function = m_timer_expired;
    m_timer_cb = cb;
    auto_restart = periodic;
    secs = period_usec / 1000000;
    t_period = ktime_set(secs, (period_usec - secs * 1000000) * 1000);
    t_start = TIMER_NOW_NS();
    hrtimer_start(&m_timer, t_period, HRTIMER_MODE_REL);
  }
}

// ------------------------------------------

void parse_argument(char* arg)
{
  struct timespec now;
  long            val;

  if (!arg) return;

  TIMER_NOW_TS(now);
  if (kstrtol(arg, 10, &val) == 0) {
    if (val > 0 && val <= 100000) {
      timer_set_rel(1000000 / val, my_callback, true);
      printk(MODULE_NAME "timer set\n");
    } else if (val > now.tv_sec && val < (now.tv_sec + 1000)) {
      now.tv_sec = val;
      timer_set_abs(now, my_callback);
      printk(MODULE_NAME "timer set\n");
    } else {
      printk(MODULE_NAME "invalid value\n");
    }
  } else {
    printk(MODULE_NAME "failed to parse input\n");
  }
}

// ------------------------------------------

static int hrtimer_dev_open(struct inode *inode, struct file *filp)
{
  return 0;
}

static int hrtimer_dev_release(struct inode *inode, struct file *filp)
{
  return 0;
}

static ssize_t hrtimer_dev_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
  if (strnlen(hrtimer_dev_data, sizeof(hrtimer_dev_data)) < count) {
    count = strnlen(hrtimer_dev_data, sizeof(hrtimer_dev_data));
  }
  // copy data from kernel space to user space
  __copy_to_user(buf, hrtimer_dev_data, count);
  hrtimer_dev_data[0] = 0;
  return count;
}

static ssize_t hrtimer_dev_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
  if (sizeof(hrtimer_dev_data) < count) {
    count = sizeof(hrtimer_dev_data) - 1;
  }
  // copy user data into kernel space
  __copy_from_user(hrtimer_dev_data, buf, count);
  hrtimer_dev_data[count] = 0;
  //printk(MODULE_NAME "received user data: '%s'\n", hrtimer_dev_data);
  parse_argument(hrtimer_dev_data);
  return count;
}

static void regist_char_device(void)
{
  // define file operations
  static struct file_operations hrtimer_dev_fops = {
    .owner   = THIS_MODULE,
    .read    = hrtimer_dev_read,
    .write   = hrtimer_dev_write,
    .open    = hrtimer_dev_open,
    .release = hrtimer_dev_release,
  };
  // dynamically allocate a major
  hrtimer_dev_major = register_chrdev(0, DEVICE_NAME, &hrtimer_dev_fops);
  if (hrtimer_dev_major < 0) {
    printk(MODULE_NAME "ERROR: cannot register the character device\n");
  } else {
    hrtimer_dev_class = class_create(THIS_MODULE, DEVICE_NAME);
    device_create(hrtimer_dev_class, NULL, MKDEV(hrtimer_dev_major, 0), NULL, DEVICE_NAME);
  }
}

static void unregister_char_device(void)
{
  unregister_chrdev(hrtimer_dev_major, DEVICE_NAME);
  device_destroy(hrtimer_dev_class, MKDEV(hrtimer_dev_major,0));
  class_unregister(hrtimer_dev_class);
  class_destroy(hrtimer_dev_class);
}

// ------------------------------------------

// kernel module initialization function
static int __init mod_init(void)
{
  printk(MODULE_NAME "module loaded\n");
  regist_char_device();

  // create the timers
  hrtimer_init(&rt_timer, CLOCK_REALTIME, HRTIMER_MODE_ABS);
  hrtimer_init(&m_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

  map_gpio();

  return 0;
}

// kernel module exit function
static void __exit mod_exit(void)
{
  unregister_char_device();

  hrtimer_cancel(&rt_timer);
  hrtimer_cancel(&m_timer);

  printk(MODULE_NAME "module removed\n");
}

module_init(mod_init);
module_exit(mod_exit);
MODULE_LICENSE("GPL");
