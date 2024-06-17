# RC Automobile (remake)

An automobile remotely controlled by an algorithm, and can be used with a radio remote control _(later)_.

## Setup

To run this project, use [m_conf.h](main/include/m_conf.h) file and change the pins and configurations to suite your need. You will also require [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html#installation).

To build and upload the firmware, run:

```bash
idf.py build flash
```

provide board port if you have multiple boards or programmers connected to your machine.
