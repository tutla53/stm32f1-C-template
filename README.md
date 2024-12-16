# STM32F103C8T6 (Blue Pill) + libopencm3 + FreeRTOS
***

### Module
- [STM32F103C8T6 Framework](https://github.com/tutla53/stm32f1-c-template.git) forked from [Warren Gay](https://github.com/ve3wwg)
- [libopencm3](https://github.com/libopencm3/libopencm3.git) branch Master
- [FreeRTOS](https://github.com/FreeRTOS/FreeRTOS-LTS.git) branch 202012-LTS
- ST Link Tools
  ```bash
  sudo apt install stlink-tools
  ```
- GCC Cross Compiler Toolchain
  ```bash
  sudo apt install gcc-arm-none-eabi
  ```
- Python
  ```bash
  sudo apt install python-is-python3
  ```
***

## Getting Started:
### Cloning the Repository
- `git clone` this repository with this command:
    ```bash
    git clone --recurse-submodules https://github.com/tutla53/stm32f1-c-template.git 
    ```
- If you didn't use a `--recursive` git clone, then you need to make
  sure that `embassy-rs` is fetched now. From the top level apply
  one of:
  ```bash
  git submodule update --init --recursive --remote  # First time
  git submodule update --recursive --remote         # Subsequent
  ```
- Update the submodules to the latest commit on it's tracked branch:
  ```bash
  git submodule update --recursive --remote 
  ```
  
### Build and Run the First Program
#### Adding the Build Target
- Let say `~` is the top level of this repository
- Move to the Directory `~/stm32f103c8t6` and then run `make`
  ```bash
  $ cd ~/stm32f103c8t6
  $ make
  ```

#### Build the Project
- Move to the Directory `~/template` and the run `make`
  ```bash
  $ cd ~/template
  $ make
  ```

#### Run the Example from ve3wwg
- You can run the example from the embassy-rs which located at `~/stm32f103c8t6/rtos/`
- Select and move to the example project e.g. `blinky` then run `make`
  ```bash
  $ cd ~/stm32f103c8t6/rtos/blinky
  $ make
  ```

