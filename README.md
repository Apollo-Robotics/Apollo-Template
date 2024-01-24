# Apollo Template

[![License: MPL 2.0](https://img.shields.io/badge/License-MPL%202.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)

[![Build](https://github.com/Apollo-Robotics/Apollo-Template-PROS/actions/workflows/main.yml/badge.svg)](https://github.com/Apollo-Robotics/Apollo-Template-PROS/blob/main/.github/workflows/main.yml)

A custom template based on the PROS (Purdue Robotics Operating System) meant to handle basic robot functions

## Usage

### Self-Installation

1. Download the latest library of the template [here](https://github.com/Apollo-Robotics/Apollo-Template-PROS/releases).
2. Open / Move the downloaded .zip file to the main directory of your active PROS project.
3. In the PROS CLI apply the template using the following commands:

```bash
prosv5 c fetch Apollo-Template@x.x.x.zip
prosv5 c apply Apollo-Template
```

&emsp;&emsp;(`x.x.x` is the version of the downloaded library that is being installed)

&emsp;&emsp;Example:

```bash
prosv5 c fetch Apollo-Template@0.0.1.zip
prosv5 c apply Apollo-Template
```

4. Once the commands have been executed and the program installed, add `#include "apollo/api.hpp"` anywhere in `main.h`
5. Once done, intialize the drivetrain in any `.cpp` or `.c` file, such as main.cpp

&emsp;&emsp;To initialize, define a chassis using the `Drivetrain` class, give it a name, and add your motor ports and drive parameters like such:

```c++
Drivetrain drivetrain(util::TANK_DRIVE, {1, 2}, {3, 4}, 5, 4, 1, 200);
```

&emsp;&emsp;Read the documentation [here](https://github.com/Apollo-Robotics/Apollo-Template-PROS/wiki) to finish initializing or look at the example definition [here](https://github.com/Apollo-Robotics/Apollo-Template-PROS/blob/main/src/main.cpp)!

6. Finally, reference the class in any `.hpp` or `.h` file, such as `main.h`, like such:

```c++
extern Drivetrain drivetrain();
```

And.... Your done!

It's not **_rocket science_**, it's **Robotics**!

### Pre-Done Installation

1. Download the latest template [here](https://github.com/Apollo-Robotics/Apollo-Template-PROS/releases).
