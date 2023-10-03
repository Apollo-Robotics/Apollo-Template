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
prosv5 c fetch Apollo@x.x.x.zip
prosv5 c apply Apollo
```

&emsp;&emsp;(`x.x.x` is the version of the downloaded library that is being installed)

&emsp;&emsp;Example:

```bash
prosv5 c fetch Apollo@0.1.0.zip
prosv5 c apply Apollo
```

4. Once the commands have been executed and the program installed, add `#include "Apollo/api.hpp"` anywhere in `main.h`
5. Once done, intialize the drivetrain in any `.cpp` or `.c` file, such as main.cpp

&emsp;&emsp;To initialize, define a chassis using the `Chassis` class and give it a name, like such:

```c++
Chassis chassis();
```

&emsp;&emsp;Read the documentation [here](https://github.com/Apollo-Robotics/Apollo-Template-PROS/wiki) to finish initializing or look at the example definition [here](https://github.com/Apollo-Robotics/Apollo-Template-PROS/blob/main/src/main.cpp)!

6. Finally, reference the class in any `.hpp` or `.h` file, such as `main.h`, like such:

```c++
extern Chassis chassis();
```

And.... Your done!

It's not **_rocket science_**, it's **Robotics**!

<!--### Pre-Done Installation-->