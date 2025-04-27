# Rust on ESP32: From C++ to Memory-Safe Embedded Systems
## A Comprehensive Course for Electrical Engineering Students

---

# Table of Contents
- [Course Overview](#course-overview)
- [Prerequisites](#prerequisites)
- [Hardware Requirements](#hardware-requirements)
- [Module 1: Environment Setup & Rust Fundamentals](#module-1-environment-setup--rust-fundamentals)
- [Module 2: Memory Management & Safety](#module-2-memory-management--safety)
- [Module 3: ESP32 Peripherals in Rust](#module-3-esp32-peripherals-in-rust)
- [Module 4: WiFi Networking](#module-4-wifi-networking)
- [Module 5: LoRa Communication](#module-5-lora-communication)
- [Module 6: Advanced Topics](#module-6-advanced-topics)
- [Module 7: Final Project](#module-7-final-project)
- [Assessment Strategy](#assessment-strategy)
- [Resources and References](#resources-and-references)

---

# Course Overview

This course introduces Rust programming language for embedded systems development on the ESP32 D0WDQ6 chip (WiFi-LoRa-32-V2 board). The course is specifically designed for electrical engineering students with prior experience in C++ embedded programming. 

By the end of this course, students will be able to:
- Program ESP32 microcontrollers using Rust instead of C/C++
- Understand Rust's memory safety and ownership model advantages in embedded contexts
- Develop secure and robust applications for the ESP32 using WiFi and LoRa capabilities
- Implement best practices for embedded Rust development
- Analyze and compare memory and performance characteristics of Rust vs C++ solutions

---

# Prerequisites

- Intermediate C++ programming skills
- Basic understanding of embedded systems concepts
- Experience with microcontroller programming (Arduino, STM32, etc.)
- Familiarity with basic electronics and digital logic
- Understanding of serial communication protocols (UART, SPI, I2C)

---

# Hardware Requirements

- ESP32 WiFi-LoRa-32-V2 development board (Heltec or compatible)
- USB-C cable for programming
- Breadboard and jumper wires
- Basic sensors and actuators:
  - Temperature/humidity sensor (DHT22 or BME280)
  - LEDs and resistors
  - Push buttons
  - Optional: OLED display (if not included on board)
- Optional: Second ESP32 board for LoRa communication projects

---

# Module 1: Environment Setup & Rust Fundamentals

## Lecture 1.1: Introduction to Embedded Rust and ESP32 Architecture

### Objectives
- Understand the differences between C++ and Rust for embedded development
- Become familiar with the ESP32 D0WDQ6 architecture and capabilities
- Learn the advantages of memory-safe programming in embedded systems

### Topics
1. **Course Introduction**
   - Course structure and expectations
   - Assessment overview
   - Resources and support

2. **Embedded Rust vs. C++: Paradigm Shift**
   - Memory safety guarantees
   - Compilation model differences
   - Zero-cost abstractions
   - Static type checking and no runtime overhead

3. **ESP32 D0WDQ6 Architecture Overview**
   - Dual-core Tensilica Xtensa LX6 processor
   - Memory layout and organization
   - Peripheral overview
   - WiFi and LoRa capabilities
   - Power management features

4. **WiFi-LoRa-32-V2 Board Components**
   - Onboard components and pinout
   - GPIO capabilities and limitations
   - OLED display and LoRa module
   - Battery management system

### Lab Exercise
- Exploration of the board components
- Identification of key pins and interfaces
- Documentation of the board's capabilities and limitations

---

## Lecture 1.2: Setting Up the Rust Toolchain for ESP32

### Objectives
- Install and configure the Rust toolchain for ESP32 development
- Understand the Rust build process for embedded targets
- Set up a development environment for ESP32 programming

### Topics
1. **Rust Installation and Setup**
   - Installing rustup and cargo
   - Adding the Xtensa target support
   - Setting up esp-rs tools and dependencies

2. **IDE Configuration**
   - VS Code setup with rust-analyzer
   - Debugging configuration
   - Recommended extensions and tools

3. **ESP-IDF and ESP-RS Integration**
   - Understanding the relationship between ESP-IDF and Rust
   - esp-idf-sys bindings
   - ESP-RS project ecosystem

4. **Project Structure and Cargo.toml Configuration**
   - Anatomy of a Rust embedded project
   - Dependencies management
   - Target-specific configurations
   - Build profiles for debug and release

### Code Example: Cargo.toml for ESP32 Project
```toml
[package]
name = "esp32-rust-project"
version = "0.1.0"
authors = ["Your Name <your.email@example.com>"]
edition = "2021"

[dependencies]
esp-idf-sys = { version = "0.32", features = ["binstart"] }
esp-idf-hal = "0.40"
esp-idf-svc = "0.43"
embedded-hal = "0.2.7"
embedded-svc = "0.23"
log = "0.4"

[build-dependencies]
embuild = "0.31"
anyhow = "1.0"

[profile.release]
opt-level = "s"
```

### Lab Exercise
- Install Rust and required toolchains
- Configure IDE with necessary extensions
- Verify the toolchain installation with a simple build test
- Explore the ESP-RS documentation and examples repository

---

## Lecture 1.3: Rust Fundamentals from an Embedded Perspective

### Objectives
- Learn Rust's syntax and basic constructs
- Understand Rust's variable declaration and type system
- Compare Rust's approach to C++ for embedded programming

### Topics
1. **Variables and Basic Types**
   - Variables vs. C++ variables (immutability by default)
   - Primitive types and their memory sizes
   - Type inference vs. explicit typing
   - Constants and static variables in embedded context

2. **Functions and Control Flow**
   - Function declaration and return values
   - Expression-based nature of Rust
   - Control flow constructs
   - Match expressions vs. switch statements

3. **Structs, Enums, and Implementations**
   - Creating structured data types
   - Methods and associated functions
   - Enums as algebraic data types
   - `Option` and `Result` for error handling

4. **Modules and Project Organization**
   - Code organization in Rust vs. C++
   - Visibility rules
   - Using external crates
   - Documentation with Rustdoc

### Code Example: C++ vs Rust Comparison
```cpp
// C++ Version
class LedController {
private:
    int pin;
    bool state;
public:
    LedController(int pin) : pin(pin), state(false) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    
    void toggle() {
        state = !state;
        digitalWrite(pin, state ? HIGH : LOW);
    }
    
    bool getState() const {
        return state;
    }
};
```

```rust
// Rust Version
struct LedController {
    pin: u8,
    state: bool,
}

impl LedController {
    fn new(pin: u8) -> Self {
        let mut controller = LedController {
            pin,
            state: false,
        };
        
        // Setup pin as output and set to LOW
        // (Implementation will use actual ESP32 GPIO APIs)
        
        controller
    }
    
    fn toggle(&mut self) {
        self.state = !self.state;
        // Toggle GPIO using ESP32 APIs
    }
    
    fn get_state(&self) -> bool {
        self.state
    }
}
```

### Lab Exercise
- Create basic Rust programs focusing on syntax and semantics
- Convert simple C++ embedded snippets to Rust
- Practice using Rust's documentation and cargo tools

---

## Lecture 1.4: First Project - Blinky with Structured Timing

### Objectives
- Implement a basic LED blink program in Rust
- Understand GPIO control in Rust on ESP32
- Implement proper timing mechanisms in Rust

### Topics
1. **GPIO Control in Rust on ESP32**
   - Using the `esp-idf-hal` crate for GPIO operations
   - Pin configuration and modes
   - Digital output control
   - Comparing with C++ GPIO control methods

2. **Timing in Embedded Rust**
   - Different delay methods (blocking vs non-blocking)
   - Using `FreeRTOS` facilities through Rust
   - Timer interrupts and callbacks
   - Comparing with C++ timing approaches

3. **Structured Project Organization**
   - Separating concerns and responsibilities
   - Creating abstraction layers for hardware
   - Error handling approaches
   - Logging and debugging techniques

4. **Building and Flashing to ESP32**
   - Cargo build process
   - Flashing tools and procedures
   - Troubleshooting common issues
   - Serial monitoring

### Code Example: LED Blink in Rust
```rust
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{Gpio2, Output, PinDriver};
use esp_idf_hal::peripherals::Peripherals;
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    // Initialize the logger
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("Starting Blinky example");

    // Get peripherals
    let peripherals = Peripherals::take()?;
    let led_pin = peripherals.pins.gpio2;
    
    // Create a pin driver for the LED
    let mut led = PinDriver::output(led_pin)?;
    
    // Blink loop
    log::info!("Entering blink loop");
    loop {
        led.set_high()?;
        log::info!("LED ON");
        FreeRtos::delay_ms(1000);
        
        led.set_low()?;
        log::info!("LED OFF");
        FreeRtos::delay_ms(1000);
    }
}
```

### Lab Exercise
- Implement an LED blink program with variable patterns
- Add a button to change the blink pattern
- Implement a timeout feature using FreeRTOS timers
- Document and compare the Rust approach with a C++ equivalent

### Project Assignment
**Enhanced Blinky**

Create a Rust program for ESP32 that controls multiple LEDs with different blink patterns. Requirements:
- Use at least 3 LEDs connected to different GPIO pins
- Implement at least 3 different blink patterns
- Use a button to cycle through patterns
- Implement proper error handling
- Add debug logging
- Document code with Rustdoc comments

---

# Module 2: Memory Management & Safety

## Lecture 2.1: Rust's Ownership Model vs C++ Memory Management

### Objectives
- Understand Rust's ownership, borrowing, and lifetime mechanisms
- Compare with C++ memory management approaches
- Learn how to apply ownership rules in embedded contexts

### Topics
1. **Ownership Rules in Rust**
   - Each value has a single owner
   - Value scope and drop semantics
   - Move semantics vs. C++ copy/move
   - Stack vs. heap allocation in embedded systems

2. **Borrowing and References**
   - Shared (&) vs. mutable (&mut) references
   - Reference rules and compile-time checking
   - Comparing with C++ pointers and references
   - Avoiding null pointer dereferences

3. **Lifetimes**
   - Explicit vs. implicit lifetimes
   - Lifetime annotations
   - Lifetime elision rules
   - Struct fields with references

4. **Implications for Embedded Development**
   - Static allocation preferences
   - Avoiding heap fragmentation
   - Predictable resource cleanup
   - Preventing common embedded bugs

### Code Example: Ownership in Practice
```rust
// This function takes ownership of the sensor object
fn configure_and_read(mut sensor: TemperatureSensor) -> (TemperatureSensor, f32) {
    // Configure sensor
    sensor.set_resolution(12);
    
    // Read value
    let temperature = sensor.read_temperature();
    
    // Return ownership of the sensor along with the reading
    (sensor, temperature)
}

// This function borrows the sensor object
fn read_only(sensor: &TemperatureSensor) -> f32 {
    sensor.read_temperature()
}

// This function borrows the sensor object mutably
fn reconfigure(sensor: &mut TemperatureSensor, resolution: u8) {
    sensor.set_resolution(resolution);
}

fn main() {
    let mut sensor = TemperatureSensor::new(/* config */);
    
    // Borrow immutably for reading
    let temp1 = read_only(&sensor);
    
    // Borrow mutably for configuration
    reconfigure(&mut sensor, 10);
    
    // Pass ownership, then get it back
    let (sensor, temp2) = configure_and_read(sensor);
    
    // We can use sensor again because we got ownership back
    let temp3 = read_only(&sensor);
}
```

### Lab Exercise
- Analyze different scenarios of memory and resource management
- Compare ownership approaches in Rust vs. C++
- Implement a resource management example with proper ownership
- Use Rust's error messages to understand ownership violations

---

## Lecture 2.2: Error Handling with Result and Option Types

### Objectives
- Learn Rust's approach to error handling without exceptions
- Understand Option and Result types and their variants
- Implement robust error handling in embedded applications

### Topics
1. **Option Type Overview**
   - Some(T) and None variants
   - Using Option instead of null pointers
   - Common methods: `is_some()`, `unwrap()`, `expect()`, `map()`
   - Pattern matching with Option

2. **Result Type for Error Handling**
   - Ok(T) and Err(E) variants
   - Error propagation with `?` operator
   - Creating custom error types
   - Converting between error types

3. **Error Handling Patterns in Embedded Systems**
   - Graceful degradation strategies
   - Recovery mechanisms
   - Logging and reporting errors
   - When to panic vs. when to handle errors

4. **Comparing with C++ Error Handling**
   - Exceptions vs. Result type
   - Error codes vs. structured errors
   - Resource cleanup guarantees
   - Performance considerations in constrained environments

### Code Example: Error Handling in Sensor Reading
```rust
use core::fmt;

// Custom error type for sensor operations
#[derive(Debug)]
enum SensorError {
    ConnectionFailed,
    InvalidReading,
    Timeout,
    CalibrationError,
}

impl fmt::Display for SensorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SensorError::ConnectionFailed => write!(f, "Failed to connect to sensor"),
            SensorError::InvalidReading => write!(f, "Sensor returned invalid reading"),
            SensorError::Timeout => write!(f, "Sensor reading timed out"),
            SensorError::CalibrationError => write!(f, "Sensor calibration error"),
        }
    }
}

struct TemperatureSensor {
    // Sensor fields
}

impl TemperatureSensor {
    // Returns a Result with either the temperature or an error
    fn read_temperature(&self) -> Result<f32, SensorError> {
        // Simulate sensor reading logic
        let connected = self.check_connection();
        if !connected {
            return Err(SensorError::ConnectionFailed);
        }
        
        // Try to read raw value with timeout
        match self.read_raw_value_with_timeout() {
            Some(raw_value) => {
                // Validate reading
                if self.is_valid_reading(raw_value) {
                    // Convert to temperature
                    Ok(self.convert_to_temperature(raw_value))
                } else {
                    Err(SensorError::InvalidReading)
                }
            },
            None => Err(SensorError::Timeout),
        }
    }
    
    // Helper methods
    fn check_connection(&self) -> bool { true /* Actual implementation */ }
    fn read_raw_value_with_timeout(&self) -> Option<u16> { Some(0) /* Actual implementation */ }
    fn is_valid_reading(&self, raw_value: u16) -> bool { true /* Actual implementation */ }
    fn convert_to_temperature(&self, raw_value: u16) -> f32 { 25.0 /* Actual implementation */ }
}

fn main() -> Result<(), SensorError> {
    let sensor = TemperatureSensor { /* initialize */ };
    
    // Using the ? operator for error propagation
    let temp = sensor.read_temperature()?;
    println!("Temperature: {:.1}°C", temp);
    
    // Alternative pattern matching approach
    match sensor.read_temperature() {
        Ok(temp) => {
            println!("Temperature: {:.1}°C", temp);
        },
        Err(SensorError::ConnectionFailed) => {
            println!("Please check sensor wiring");
            // Try recovery action
        },
        Err(SensorError::Timeout) => {
            println!("Sensor reading timed out. Retrying...");
            // Retry logic
        },
        Err(e) => {
            println!("Sensor error: {}", e);
            // General error handling
        }
    }
    
    Ok(())
}
```

### Lab Exercise
- Implement error handling in a sensor reading application
- Create custom error types for different failure modes
- Practice error propagation and recovery strategies
- Compare error handling approaches with C++ equivalents

---

## Lecture 2.3: No Heap? No Problem - Working with #[no_std]

### Objectives
- Understand bare-metal Rust programming without the standard library
- Learn strategies for embedded development without heap allocation
- Implement memory-efficient solutions for constrained environments

### Topics
1. **Understanding #[no_std] Development**
   - Standard library vs. core library
   - Available features without std
   - Setting up a #[no_std] project
   - ESP32 specific considerations

2. **Static Memory Allocation Strategies**
   - Const and static initializations
   - Fixed-size buffers
   - Array-based collections
   - Stack allocation constraints

3. **Alternatives to Heap-Based Collections**
   - heapless crate introduction
   - Fixed-capacity vectors, strings, and maps
   - Embedded-specific collections

4. **Singleton Patterns for Global State**
   - Safe global state with `static mut`
   - `core::cell` types for interior mutability
   - Critical sections and interrupt safety
   - Mutex and RefCell implementations

### Code Example: No-Heap Sensor Data Collection
```rust
#![no_std]
#![no_main]

use core::cell::RefCell;
use critical_section::Mutex;
use esp32_hal::{clock::ClockControl, peripherals::Peripherals, prelude::*, timer::TimerGroup};
use esp_backtrace as _;
use heapless::Vec;

// Fixed capacity collection for sensor readings
type ReadingsBuffer = Vec<i16, 64>; // Stores up to 64 readings

// Global storage for sensor readings
static SENSOR_READINGS: Mutex<RefCell<ReadingsBuffer>> = Mutex::new(RefCell::new(Vec::new()));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    
    // Setup timer for periodic readings
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;
    
    // Configure ADC for sensor readings
    let mut adc = peripherals.ADC1.configure();
    let mut sensor_pin = peripherals.pins.gpio34.into_analog_adc();
    
    // Main loop for sensor readings
    loop {
        // Read sensor value
        let raw_value = adc.read(&mut sensor_pin).unwrap_or(0);
        
        // Store in our global buffer using a critical section
        critical_section::with(|cs| {
            let mut readings = SENSOR_READINGS.borrow_ref_mut(cs);
            if readings.is_full() {
                // Handle full buffer - in this case, remove the oldest reading
                readings.pop();
            }
            // Add new reading
            let _ = readings.push(raw_value);
        });
        
        // Process readings periodically
        if timer0.wait().is_ok() {
            process_readings();
        }
    }
}

fn process_readings() {
    critical_section::with(|cs| {
        let readings = SENSOR_READINGS.borrow(cs);
        
        // Calculate average
        if !readings.is_empty() {
            let sum: i32 = readings.iter().map(|&v| v as i32).sum();
            let avg = sum / readings.len() as i32;
            
            // Do something with the average
            // (e.g., trigger alarm, update display, etc.)
        }
    });
}
```

### Lab Exercise
- Create a #[no_std] project for ESP32
- Implement a sensor data collection system using fixed-capacity collections
- Develop a circular buffer for storing sensor history
- Compare memory usage with equivalent heap-based solutions

---

## Lecture 2.4: Project - Robust Sensor Hub

### Objectives
- Apply ownership, error handling, and #[no_std] concepts
- Create a robust multi-sensor data collection system
- Implement proper error recovery strategies

### Topics
1. **Project Architecture Overview**
   - Sensor abstraction design
   - Data collection and processing pipeline
   - Error handling strategy
   - Memory management approach

2. **Implementing Sensor Traits**
   - Creating a common sensor interface
   - Trait implementation for different sensor types
   - Error type hierarchies
   - Static vs. dynamic polymorphism

3. **Building the Sensor Hub**
   - Managing multiple sensors
   - Scheduling sensor readings
   - Processing and aggregating data
   - Handling various error conditions

4. **Testing and Verification**
   - Unit testing strategies for embedded Rust
   - Mocking hardware dependencies
   - Validating error handling paths
   - Measuring resource usage

### Project Specification: Robust Sensor Hub

**Requirements**
1. Support at least two sensor types (e.g., temperature, humidity, pressure)
2. Implement proper error handling for all sensor operations
3. Use no heap allocations (#[no_std] approach)
4. Provide graceful degradation when sensors fail
5. Implement sensor calibration functions
6. Create a simple user interface (LED indicators or OLED display)

**Deliverables**
1. Complete Rust project with proper documentation
2. Architecture document explaining design decisions
3. Error handling strategy document
4. Memory usage analysis

---

# Module 3: ESP32 Peripherals in Rust

## Lecture 3.1: GPIO Control and Interrupt Handling

### Objectives
- Master GPIO control using the esp-idf-hal crate
- Implement proper interrupt handling in Rust
- Understand safe concurrency in interrupt contexts

### Topics
1. **Advanced GPIO Operations**
   - Input and output pin configurations
   - Pull-up and pull-down resistors
   - Drive strength and slew rate control
   - Special function pins

2. **Interrupt Configuration**
   - Edge vs. level-triggered interrupts
   - Rising/falling edge detection
   - Interrupt priority levels
   - Interrupt service routines in Rust

3. **Safe Interrupt Handling**
   - Critical sections and atomic operations
   - Safe shared state access
   - Interrupt safety considerations
   - Avoiding deadlocks and race conditions

4. **Debouncing and Filtering**
   - Software debouncing techniques
   - Hardware filtering options
   - Efficient implementation approaches
   - Testing and validation

### Code Example: Interrupt-Driven Button Handling
```rust
use esp_idf_hal::gpio::{Gpio0, Input, PinDriver, Pull, InterruptType};
use esp_idf_hal::interrupt::{self, CriticalSection, InterruptHandler};
use esp_idf_hal::peripherals::Peripherals;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

// Shared state between ISR and main thread
static BUTTON_PRESSED: AtomicBool = AtomicBool::new(false);
static LAST_PRESS_TIME: std::sync::Mutex<Option<Instant>> = std::sync::Mutex::new(None);

// Interrupt handler function
fn button_isr(_: *mut core::ffi::c_void) {
    // Use relaxed ordering for simple flag
    BUTTON_PRESSED.store(true, Ordering::Relaxed);
    
    // Update last press time with debounce logic
    let now = Instant::now();
    let mut last_time = LAST_PRESS_TIME.lock().unwrap();
    
    if let Some(prev_time) = *last_time {
        // Debounce: ignore presses less than 50ms apart
        if now.duration_since(prev_time) < Duration::from_millis(50) {
            return;
        }
    }
    
    *last_time = Some(now);
}

fn main() -> anyhow::Result<()> {
    // Initialize the ESP-IDF HAL
    esp_idf_sys::link_patches();
    
    // Get peripherals
    let peripherals = Peripherals::take()?;
    
    // Configure GPIO0 as input with pull-up
    let mut button = PinDriver::input(peripherals.pins.gpio0)?;
    button.set_pull(Pull::Up)?;
    
    // Register interrupt handler
    button.set_interrupt_type(InterruptType::NegEdge)?;
    unsafe {
        button.subscribe(button_isr)?;
    }
    
    // Enable the interrupt
    button.enable_interrupt()?;
    
    // Main loop
    loop {
        // Check if button was pressed
        if BUTTON_PRESSED.swap(false, Ordering::Relaxed) {
            println!("Button pressed!");
            
            // Do something in response to button press
            // ...
            
            // Small delay to prevent CPU hogging
            std::thread::sleep(Duration::from_millis(10));
        }
    }
}
```

### Lab Exercise
- Implement an interrupt-driven button controller
- Create a GPIO-based state machine triggered by interrupts
- Compare polling vs. interrupt approaches for button detection
- Measure and optimize interrupt response time

---

## Lecture 3.2: SPI and I2C Communication

### Objectives
- Implement SPI and I2C device communication in Rust
- Create abstractions for common sensors and displays
- Understand the performance characteristics of different protocols

### Topics
1. **SPI Protocol Implementation**
   - SPI bus configuration in Rust
   - Chip select management
   - Transaction-based API
   - Multiple device handling

2. **I2C Protocol Implementation**
   - I2C bus setup and addressing
   - Register-based device communication
   - Clock stretching and timing considerations
   - Error handling and recovery

3. **Sensor Abstraction Patterns**
   - Creating device drivers in Rust
   - Register maps and bitfield manipulation
   - Command sequences and state management
   - Testing and simulation strategies

4. **Display Integration**
   - OLED display control via SPI/I2C
   - Graphics primitives
   - Text rendering
   - Memory-efficient display buffer management

### Code Example: I2C Sensor Driver Implementation
```rust
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use esp_idf_hal::delay::Ets;
use esp_idf_hal::i2c::{I2cConfig, I2cDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::units::Hertz;

const BME280_ADDR: u8 = 0x76;
const REG_CHIP_ID: u8 = 0xD0;
const REG_RESET: u8 = 0xE0;
const REG_CTRL_HUM: u8 = 0xF2;
const REG_STATUS: u8 = 0xF3;
const REG_CTRL_MEAS: u8 = 0xF4;
const REG_CONFIG: u8 = 0xF5;
const REG_PRESS: u8 = 0xF7;
const REG_TEMP: u8 = 0xFA;
const REG_HUM: u8 = 0xFD;

#[derive(Debug)]
enum Bme280Error {
    I2cError,
    InvalidChipId,
    ConfigurationError,
    MeasurementTimeout,
}

struct Bme280<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C, E> Bme280<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E> + Read<Error = E>,
{
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }
    
    pub fn init(&mut self) -> Result<(), Bme280Error> {
        // Verify chip ID
        let chip_id = self.read_register(REG_CHIP_ID).map_err(|_| Bme280Error::I2cError)?;
        if chip_id != 0x60 {
            return Err(Bme280Error::InvalidChipId);
        }
        
        // Reset the device
        self.write_register(REG_RESET, 0xB6).map_err(|_| Bme280Error::I2cError)?;
        
        // Wait for reset to complete
        let mut timeout = 100;
        while self.read_register(REG_STATUS).map_err(|_| Bme280Error::I2cError)? & 0x01 != 0 {
            Ets::delay_ms(10);
            timeout -= 1;
            if timeout == 0 {
                return Err(Bme280Error::MeasurementTimeout);
            }
        }
        
        // Configure humidity oversampling
        self.write_register(REG_CTRL_HUM, 0x01).map_err(|_| Bme280Error::I2cError)?; // x1 oversampling
        
        // Configure temperature and pressure oversampling, and mode
        // 0x27 = 0b00100111
        // - Temperature oversampling x1
        // - Pressure oversampling x1
        // - Normal mode
        self.write_register(REG_CTRL_MEAS, 0x27).map_err(|_| Bme280Error::I2cError)?;
        
        // Configure standby and filter
        // 0x50 = 0b01010000
        // - Standby time 500ms
        // - Filter coefficient 16
        self.write_register(REG_CONFIG, 0x50).map_err(|_| Bme280Error::I2cError)?;
        
        Ok(())
    }
    
    pub fn read_temperature(&mut self) -> Result<f32, Bme280Error> {
        let mut buffer = [0u8; 3];
        self.i2c
            .write_read(self.address, &[REG_TEMP], &mut buffer)
            .map_err(|_| Bme280Error::I2cError)?;
            
        // Raw temperature value (simplified, actual implementation would use calibration)
        let raw_temp = (buffer[0] as u32) << 16 | (buffer[1] as u32) << 8 | (buffer[2] as u32);
        let raw_temp = raw_temp >> 4; // 20-bit value
        
        // Convert to temperature in degrees C (simplified)
        // In a real implementation, we would use the calibration coefficients
        // and the compensation formula from the datasheet
        let temp = (raw_temp as f32) / 100.0;
        
        Ok(temp)
    }
    
    // Helper methods
    fn read_register(&mut self, register: u8) -> Result<u8, E> {
        let mut buffer = [0];
        self.i2c.write_read(self.address, &[register], &mut buffer)?;
        Ok(buffer[0])
    }
    
    fn write_register(&mut self, register: u8, value: u8) -> Result<(), E> {
        self.i2c.write(self.address, &[register, value])
    }
}

fn main() -> anyhow::Result<()> {
    // Initialize ESP-IDF
    esp_idf_sys::link_patches();
    
    // Acquire peripherals
    let peripherals = Peripherals::take()?;
    
    // Configure I2C
    let i2c = peripherals.i2c0;
    let sda = peripherals.pins.gpio21;
    let scl = peripherals.pins.gpio22;
    
    let config = I2cConfig::new().baudrate(Hertz(400_000));
    let i2c = I2cDriver::new(i2c, sda, scl, &config)?;
    
    // Create and initialize sensor
    let mut bme280 = Bme280::new(i2c, BME280_ADDR);
    bme280.init()?;
    
    // Read temperature
    loop {
        match bme280.read_temperature() {
            Ok(temp) => println!("Temperature: {:.2}°C", temp),
            Err(e) => println!("Error reading temperature: {:?}", e),
        }
        
        Ets::delay_ms(2000);
    }
}
```

### Lab Exercise
- Implement an I2C driver for a temperature/humidity sensor
- Create an SPI driver for an external display
- Compare performance and resource usage between SPI and I2C
- Build a sensor fusion application using multiple communication protocols

---

## Lecture 3.3: ADC, DAC, and PWM Control

### Objectives
- Implement analog-to-digital and digital-to-analog conversion in Rust
- Master PWM signal generation for motor and LED control
- Understand calibration and noise reduction techniques

### Topics
1. **ADC Configuration and Usage**
   - ADC channel setup
   - Resolution and sampling rate settings
   - Multi-channel scanning
   - Calibration and accuracy considerations

2. **DAC Operation**
   - DAC channel configuration
   - Waveform generation
   - Output buffering and filtering
   - Noise reduction techniques

3. **PWM Signal Generation**
   - LEDC peripheral configuration
   - Frequency and duty cycle control
   - Fading and transition effects
   - Motor speed control applications

4. **Analog Signal Processing**
   - Signal filtering techniques
   - Oversampling and averaging
   - Moving average and other filtering algorithms
   - Converting raw values to meaningful units

### Code Example: PWM LED Control
```rust
use esp_idf_hal::ledc::{config::TimerConfig, Channel, Timer, LEDC};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::*;
use esp_idf_hal::units::Hertz;
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    // Initialize the ESP-IDF HAL
    esp_idf_sys::link_patches();
    
    // Get peripherals
    let peripherals = Peripherals::take()?;
    let ledc = LEDC::new(peripherals.ledc)?;
    let timer = ledc.get_timer(Timer::Timer0)?;
    
    // Configure timer with 5kHz frequency and 13-bit resolution
    let timer_config = TimerConfig::new()
        .frequency(Hertz(5000))
        .resolution(esp_idf_hal::ledc::Resolution::Bits13);
    let timer = ledc.configure_timer(timer, &timer_config)?;
    
    // Set up a channel for PWM on GPIO2
    let mut channel = ledc.get_channel(Channel::Channel0)?;
    let led = peripherals.pins.gpio2;
    let mut led_driver = ledc.configure_channel(
        &timer,
        channel,
        led,
        &esp_idf_hal::ledc::config::ChannelConfig::default()
    )?;
    
    // Set the initial duty cycle to 0%
    led_driver.set_duty(0)?;
    
    // Create a breathing effect
    let max_duty = (1 << 13) - 1; // Max duty for 13-bit resolution
    loop {
        // Increase brightness (fade in)
        for duty in (0..=max_duty).step_by(100) {
            led_driver.set_duty(duty)?;
            std::thread::sleep(Duration::from_millis(10));
        }
        
        // Hold at full brightness
        std::thread::sleep(Duration::from_millis(500));
        
        // Decrease brightness (fade out)
        for duty in (0..=max_duty).rev().step_by(100) {
            led_driver.set_duty(duty)?;
            std::thread::sleep(Duration::from_millis(10));
        }
        
        // Hold at no brightness
        std::thread::sleep(Duration::from_millis(500));
    }
}
```

### Lab Exercise
- Implement a light dimmer using PWM
- Create an analog sensor reader with filtering
- Develop a waveform generator using DAC
- Build a motor speed controller using PWM

---

## Lecture 3.4: Project - Digital Sensor Integration

### Objectives
- Integrate multiple digital sensors into a cohesive system
- Implement proper error handling and recovery mechanisms
- Create a user interface to display sensor data

### Topics
1. **Project Architecture**
   - Sensor abstraction layers
   - Data acquisition pipeline
   - Processing and filtering algorithms
   - User interface design

2. **Implementation Approach**
   - Module organization
   - Error handling strategy
   - Resource management plan
   - Testing methodology

3. **System Integration**
   - Inter-sensor synchronization
   - Power management considerations
   - Data storage and retrieval
   - User interface implementation

4. **Performance Optimization**
   - Minimizing memory usage
   - Reducing power consumption
   - Optimizing CPU utilization
   - Balancing responsiveness and accuracy

### Project Specification: Environmental Monitoring Station

**Requirements**
1. Integrate at least three sensors:
   - Temperature/humidity sensor (e.g., DHT22, BME280)
   - Light level sensor (e.g., BH1750, TSL2561)
   - Air quality sensor (e.g., CCS811, SGP30)
2. Display current readings and trends on OLED display
3. Implement adaptive sampling rates based on conditions
4. Create warning indicators for out-of-range values
5. Include calibration functions for each sensor
6. Implement power-saving modes

**Deliverables**
1. Complete Rust project with proper documentation
2. Circuit diagram and connection details
3. User manual with operation instructions
4. Test report with accuracy measurements

---

# Module 4: WiFi Networking

## Lecture 4.1: WiFi Setup and Management

### Objectives
- Configure and manage WiFi connections on ESP32
- Implement robust connection management and recovery
- Understand power consumption implications of WiFi operations

### Topics
1. **WiFi Hardware Overview**
   - ESP32 WiFi capabilities
   - Radio and antenna configuration
   - Power consumption characteristics
   - Co-existence with Bluetooth

2. **WiFi Station Mode**
   - Connecting to access points
   - WPA/WPA2/WPA3 security
   - Connection monitoring and recovery
   - DHCP and static IP configuration

3. **WiFi Access Point Mode**
   - Creating a soft access point
   - Client management
   - Security configuration
   - Captive portal implementation

4. **WiFi Scanning and Analysis**
   - Network discovery
   - Signal strength analysis
   - Channel utilization
   - Selecting optimal networks

### Code Example: WiFi Station Mode
```rust
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{BlockingWifi, ClientConfiguration, Configuration, EspWifi};
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    // Initialize ESP-IDF
    esp_idf_sys::link_patches();
    
    // Configure logging
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!("Starting WiFi example");
    
    // Get peripherals and event loop
    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    
    // Initialize WiFi
    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs))?,
        sys_loop,
    )?;
    
    // Configure WiFi in station mode
    let wifi_configuration = Configuration::Client(ClientConfiguration {
        ssid: "YourNetworkName".into(),
        password: "YourPassword".into(),
        ..Default::default()
    });
    
    // Set the configuration
    wifi.set_configuration(&wifi_configuration)?;
    
    // Start WiFi
    wifi.start()?;
    log::info!("WiFi started");
    
    // Connect to WiFi network
    log::info!("Connecting to WiFi network...");
    wifi.connect()?;
    
    // Wait for connection
    wifi.wait_netif_up()?;
    log::info!("WiFi connected!");
    
    // Get connection details
    let ip_info = wifi.wifi().sta_netif().get_ip_info()?;
    log::info!("IP info: {:?}", ip_info);
    
    // Main program loop
    loop {
        // Check WiFi status periodically
        if !wifi.is_connected()? {
            log::warn!("WiFi connection lost. Reconnecting...");
            match wifi.connect() {
                Ok(_) => {
                    if wifi.wait_netif_up().is_ok() {
                        log::info!("WiFi reconnected successfully");
                    }
                }
                Err(e) => {
                    log::error!("Failed to reconnect: {:?}", e);
                }
            }
        }
        
        // Do other tasks
        // ...
        
        std::thread::sleep(Duration::from_secs(5));
    }
}
```

### Lab Exercise
- Implement a WiFi scanner to display available networks
- Create a connection manager with automatic reconnection
- Develop a signal strength monitoring tool
- Build a configuration portal for WiFi settings

---

## Lecture 4.2: HTTP Client Implementation

### Objectives
- Implement HTTP client functionality in Rust
- Create robust API clients for external services
- Handle common networking issues and failures

### Topics
1. **HTTP Client Basics**
   - Request and response handling
   - Header management
   - Content types and encoding
   - Connection pooling

2. **REST API Communication**
   - JSON serialization and deserialization
   - Authentication methods
   - Rate limiting and backoff strategies
   - Error handling and recovery

3. **HTTPS and Security**
   - Certificate validation
   - SSL/TLS configurations
   - Secure communication practices
   - Security considerations for IoT

4. **Efficient API Design**
   - Minimizing data transfer
   - Batching requests
   - Compression techniques
   - Connection keep-alive

### Code Example: REST API Client
```rust
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{BlockingWifi, ClientConfiguration, Configuration, EspWifi};
use esp_idf_svc::http::client::{Configuration as HttpConfiguration, EspHttpConnection};
use embedded_svc::http::client::Client as HttpClient;
use embedded_svc::io::Read;
use serde::{Deserialize, Serialize};
use core::time::Duration;

// Data structures for API
#[derive(Debug, Deserialize)]
struct WeatherResponse {
    main: WeatherMain,
    name: String,
}

#[derive(Debug, Deserialize)]
struct WeatherMain {
    temp: f32,
    humidity: u8,
    pressure: u32,
}

#[derive(Debug, Serialize)]
struct SensorData {
    device_id: String,
    temperature: f32,
    humidity: u8,
}

fn main() -> anyhow::Result<()> {
    // Initialize ESP-IDF
    esp_idf_sys::link_patches();
    
    // Configure logging
    esp_idf_svc::log::EspLogger::initialize_default();
    
    // Connect to WiFi (using previous example code)
    // ...
    
    // Create HTTP client
    let http_config = HttpConfiguration {
        timeout: Some(Duration::from_secs(10)),
        ..Default::default()
    };
    
    let client = HttpClient::wrap(EspHttpConnection::new(&http_config)?);
    
    // Example: GET request to weather API
    log::info!("Making GET request to weather API");
    let weather_url = "http://api.openweathermap.org/data/2.5/weather?q=London&appid=YOUR_API_KEY&units=metric";
    
    let mut response = client.get(weather_url)?.submit()?;
    let status = response.status();
    log::info!("Response status: {}", status);
    
    // Read the response body
    let mut buffer = [0u8; 2048];
    let mut cursor = 0;
    
    while let Ok(size) = response.read(&mut buffer[cursor..]) {
        if size == 0 {
            break;
        }
        cursor += size;
        if cursor >= buffer.len() {
            break;
        }
    }
    
    // Parse JSON response
    if status == 200 {
        match serde_json::from_slice::<WeatherResponse>(&buffer[0..cursor]) {
            Ok(weather) => {
                log::info!("Weather in {}: {}°C, {}% humidity", 
                    weather.name, 
                    weather.main.temp, 
                    weather.main.humidity
                );
            },
            Err(e) => {
                log::error!("Failed to parse response: {:?}", e);
            }
        }
    } else {
        log::error!("Request failed with status {}", status);
    }
    
    // Example: POST request to send sensor data
    log::info!("Making POST request with sensor data");
    let sensor_data = SensorData {
        device_id: "esp32-001".to_string(),
        temperature: 23.5,
        humidity: 48,
    };
    
    let json_data = serde_json::to_string(&sensor_data)?;
    let api_url = "https://your-api-endpoint.com/data";
    
    let mut request = client.post(api_url)?;
    request.header("Content-Type", "application/json")?;
    
    let mut response = request.submit(json_data.as_bytes())?;
    let status = response.status();
    
    log::info!("POST response status: {}", status);
    
    // Clean up
    response.drain()?;
    
    Ok(())
}
```

### Lab Exercise
- Create a weather station application using a public API
- Implement error handling and retry logic for unreliable connections
- Build a data logging service that sends sensor data to a cloud API
- Develop a firmware update check system using HTTP

---

## Lecture 4.3: HTTP Server Implementation

### Objectives
- Create an embedded web server on ESP32 using Rust
- Implement RESTful API endpoints for device control
- Serve web interface for configuration and monitoring

### Topics
1. **HTTP Server Basics**
   - Server initialization and configuration
   - Route handling and middleware
   - Request parsing and response generation
   - Static file serving

2. **RESTful API Design**
   - Resource-oriented architecture
   - CRUD operations implementation
   - Status codes and error responses
   - API versioning strategies

3. **WebSocket for Real-time Updates**
   - WebSocket server implementation
   - Event-based communication
   - Data streaming techniques
   - Handling multiple clients

4. **Web Interface Integration**
   - HTML, CSS, and JavaScript serving
   - Browser compatibility considerations
   - Responsive design for mobile access
   - Progressive enhancement techniques

### Code Example: HTTP Server
```rust
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{BlockingWifi, AccessPointConfiguration, Configuration, EspWifi};
use esp_idf_svc::http::server::{Configuration as HttpConfiguration, EspHttpServer};
use embedded_svc::http::Method;
use embedded_svc::http::server::Request;
use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex};

// Data structures for API
#[derive(Debug, Clone, Serialize)]
struct SensorData {
    temperature: f32,
    humidity: u8,
    timestamp: u64,
}

#[derive(Debug, Deserialize)]
struct LedControl {
    status: bool,
}

fn main() -> anyhow::Result<()> {
    // Initialize ESP-IDF
    esp_idf_sys::link_patches();
    
    // Configure logging
    esp_idf_svc::log::EspLogger::initialize_default();
    
    // Create sensor data storage
    let sensor_data = Arc::new(Mutex::new(SensorData {
        temperature: 21.0,
        humidity: 50,
        timestamp: 0,
    }));
    
    // Create LED status storage
    let led_status = Arc::new(Mutex::new(false));
    
    // Set up WiFi in AP mode
    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    
    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs))?,
        sys_loop,
    )?;
    
    let wifi_configuration = Configuration::AccessPoint(AccessPointConfiguration {
        ssid: "ESP32-Rust-AP".into(),
        password: "password123".into(),
        ..Default::default()
    });
    
    wifi.set_configuration(&wifi_configuration)?;
    wifi.start()?;
    log::info!("WiFi AP started");
    
    // Create HTTP server
    let http_config = HttpConfiguration::default();
    let mut server = EspHttpServer::new(&http_config)?;
    
    // Clone shared state for handlers
    let sensor_data_clone = sensor_data.clone();
    let led_status_clone = led_status.clone();
    
    // Set up API endpoint for sensor data
    server.fn_handler("/api/sensor", Method::Get, move |_req| {
        let data = sensor_data_clone.lock().unwrap();
        let json = serde_json::to_string(&*data)?;
        
        Ok(resp)
            .status(200)
            .header("Content-Type", "application/json")
            .body(json.as_bytes())
    })?;
    
    // Set up API endpoint for LED control
    server.fn_handler("/api/led", Method::Post, move |mut req| {
        let mut buf = [0u8; 100];
        let bytes_read = req.read(&mut buf)?;
        
        if let Ok(led_control) = serde_json::from_slice::<LedControl>(&buf[..bytes_read]) {
            *led_status_clone.lock().unwrap() = led_control.status;
            
            // Actually control the LED here
            // ...
            
            Ok(resp)
                .status(200)
                .header("Content-Type", "application/json")
                .body(b"{\"success\": true}")
        } else {
            Ok(resp)
                .status(400)
                .header("Content-Type", "application/json")
                .body(b"{\"error\": \"Invalid request format\"}")
        }
    })?;
    
    // Serve static HTML page
    server.fn_handler("/", Method::Get, |_req| {
        const INDEX_HTML: &str = r#"
        <!DOCTYPE html>
        <html>
        <head>
            <title>ESP32 Rust Control Panel</title>
            <meta name="viewport" content="width=device-width, initial-scale=1">
            <style>
                body { font-family: Arial, sans-serif; margin: 0; padding: 20px; }
                .card { background: #f8f9fa; border-radius: 10px; padding: 20px; margin-bottom: 20px; }
                button { background: #0d6efd; color: white; border: none; padding: 10px 15px; border-radius: 5px; cursor: pointer; }
                button:hover { background: #0b5ed7; }
            </style>
        </head>
        <body>
            <div class="card">
                <h2>Sensor Data</h2>
                <p>Temperature: <span id="temp">--</span>°C</p>
                <p>Humidity: <span id="humidity">--</span>%</p>
                <p>Last update: <span id="timestamp">--</span></p>
            </div>
            
            <div class="card">
                <h2>LED Control</h2>
                <button id="led-toggle">Turn LED ON</button>
            </div>
            
            <script>
                let ledStatus = false;
                
                // Update sensor data
                function updateSensorData() {
                    fetch('/api/sensor')
                        .then(response => response.json())
                        .then(data => {
                            document.getElementById('temp').textContent = data.temperature.toFixed(1);
                            document.getElementById('humidity').textContent = data.humidity;
                            document.getElementById('timestamp').textContent = new Date(data.timestamp * 1000).toLocaleTimeString();
                        })
                        .catch(error => console.error('Error fetching sensor data:', error));
                }
                
                // LED toggle handler
                document.getElementById('led-toggle').addEventListener('click', function() {
                    ledStatus = !ledStatus;
                    this.textContent = ledStatus ? 'Turn LED OFF' : 'Turn LED ON';
                    
                    fetch('/api/led', {
                        method: 'POST',
                        headers: {
                            'Content-Type': 'application/json',
                        },
                        body: JSON.stringify({ status: ledStatus })
                    })
                    .then(response => response.json())
                    .catch(error => console.error('Error toggling LED:', error));
                });
                
                // Update sensor data every 5 seconds
                updateSensorData();
                setInterval(updateSensorData, 5000);
            </script>
        </body>
        </html>
        "#;
        
        Ok(resp)
            .status(200)
            .header("Content-Type", "text/html")
            .body(INDEX_HTML.as_bytes())
    })?;
    
    // Start a background task to update sensor data periodically
    std::thread::spawn(move || {
        let mut counter = 0u64;
        loop {
            // In a real application, this would read from actual sensors
            let temp = 20.0 + (counter % 10) as f32 / 10.0;
            let humidity = 50 + (counter % 20) as u8;
            
            // Update the shared sensor data
            let mut data = sensor_data.lock().unwrap();
            data.temperature = temp;
            data.humidity = humidity;
            data.timestamp = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs();
            
            counter += 1;
            std::thread::sleep(std::time::Duration::from_secs(5));
        }
    });
    
    log::info!("HTTP server started");
    log::info!("Access the web interface at http://192.168.4.1");
    
    // Keep the main thread alive
    loop {
        std::thread::sleep(std::time::Duration::from_secs(10));
    }
}
```

### Lab Exercise
- Create a web-based configuration portal for WiFi settings
- Implement a RESTful API for sensor data access
- Develop a real-time dashboard with WebSockets
- Build a file manager for accessing data logs

---

## Lecture 4.4: Project - Weather Station with Web Dashboard

### Objectives
- Create a complete IoT weather station with web interface
- Implement proper error handling and recovery for network operations
- Optimize for both performance and power efficiency

### Topics
1. **System Architecture**
   - Sensor integration
   - Data processing pipeline
   - Storage and caching strategy
   - User interface design

2. **Network Communication**
   - Local web server
   - Cloud data synchronization
   - Offline operation fallback
   - Security considerations

3. **User Interface Design**
   - Responsive web dashboard
   - Data visualization techniques
   - Configuration interface
   - Mobile compatibility

4. **Performance Optimization**
   - Memory usage reduction
   - Power consumption strategies
   - Network efficiency techniques
   - Response time optimization

### Project Specification: IoT Weather Station

**Requirements**
1. Integrate environmental sensors:
   - Temperature and humidity
   - Barometric pressure
   - Light level
   - Optional: Air quality, rain detection

2. Implement a web dashboard with:
   - Current conditions display
   - Historical data charts
   - Configuration interface
   - Alerts for extreme conditions

3. Create network capabilities:
   - Local access point when no WiFi available
   - Connection to home WiFi when available
   - Optional: Data synchronization to cloud service

4. Develop power management features:
   - Sleep modes for battery operation
   - Adaptive sampling rates
   - Low-power indicators

**Deliverables**
1. Complete Rust project with proper documentation
2. Web interface HTML, CSS, and JavaScript files
3. User manual with setup and operation instructions
4. Hardware configuration guide
5. Power consumption analysis

---

# Module 5: LoRa Communication

## Lecture 5.1: LoRa Protocol Fundamentals

### Objectives
- Understand LoRa technology and its applications
- Learn the physical and protocol characteristics of LoRa
- Configure LoRa parameters for optimal performance

### Topics
1. **LoRa Technology Overview**
   - Spread spectrum modulation
   - Frequency bands and regulations
   - Range vs. bandwidth tradeoffs
   - Battery life considerations

2. **LoRa Parameters**
   - Spreading factor
   - Bandwidth
   - Coding rate
   - Transmission power
   - Parameter optimization strategies

3. **LoRaWAN vs. Raw LoRa**
   - Protocol comparison
   - Application scenarios
   - Network architectures
   - Security considerations

4. **WiFi-LoRa-32-V2 Hardware**
   - SX1276/SX1278 transceiver
   - Antenna considerations
   - Hardware interfaces
   - Power consumption characteristics

### Code Example: LoRa Hardware Initialization
```rust
use esp_idf_hal::gpio::{Gpio18, Gpio19, Gpio23, Gpio5, Gpio27, Output, PinDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi::{SpiConfig, SpiDeviceConfig, SpiDriver, SPI2};
use esp_idf_hal::prelude::*;
use sx127x_lora::LoRa;

// LoRa pins for WiFi-LoRa-32-V2 board
const LORA_CS_PIN: i32 = 18;   // GPIO18
const LORA_RESET_PIN: i32 = 14; // GPIO14
const LORA_MISO_PIN: i32 = 19;  // GPIO19
const LORA_MOSI_PIN: i32 = 27;  // GPIO27
const LORA_SCK_PIN: i32 = 5;    // GPIO5
const LORA_DIO0_PIN: i32 = 26;  // GPIO26

// LoRa operating parameters
const FREQUENCY: i64 = 868_000_000; // 868 MHz
const SPREADING_FACTOR: u8 = 7;
const BANDWIDTH: i64 = 125_000; // 125 kHz
const CODING_RATE: u8 = 5;  // 4/5
const PREAMBLE_LEN: u16 = 8;
const TX_POWER: i8 = 17; // dBm

fn main() -> anyhow::Result<()> {
    // Initialize ESP-IDF
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    
    log::info!("Initializing LoRa module");
    
    // Get peripherals
    let peripherals = Peripherals::take()?;
    
    // Configure SPI
    let spi = peripherals.spi2;
    let sclk = peripherals.pins.gpio5;
    let miso = peripherals.pins.gpio19;
    let mosi = peripherals.pins.gpio27;
    let cs = peripherals.pins.gpio18;
    
    // Reset pin
    let reset_pin = PinDriver::output(peripherals.pins.gpio14)?;
    
    // DIO0 pin for interrupts
    let dio0_pin = peripherals.pins.gpio26;
    
    // Configure SPI
    let spi_config = SpiConfig::new()
        .baudrate(10.MHz().into());
    
    let spi_driver = SpiDriver::new(
        spi,
        sclk,
        mosi,
        Some(miso),
        &spi_config
    )?;
    
    // SPI device config for LoRa module
    let device_config = SpiDeviceConfig::new()
        .cs_pin(Some(cs))
        .mode(embedded_hal::spi::MODE_0)
        .clock_speed(10.MHz().into());
    
    let spi_device = spi_driver.device(device_config)?;
    
    // Initialize LoRa module
    let mut lora = match LoRa::new(spi_device, reset_pin, dio0_pin) {
        Ok(lora) => {
            log::info!("SX127x initialized successfully");
            lora
        },
        Err(e) => {
            log::error!("Failed to initialize SX127x: {:?}", e);
            return Err(anyhow::anyhow!("Failed to initialize SX127x"));
        }
    };
    
    // Configure LoRa parameters
    lora.set_frequency(FREQUENCY)?;
    lora.set_spreading_factor(SPREADING_FACTOR)?;
    lora.set_bandwidth(BANDWIDTH)?;
    lora.set_coding_rate(CODING_RATE)?;
    lora.set_preamble_length(PREAMBLE_LEN)?;
    lora.set_tx_power(TX_POWER)?;
    
    log::info!("LoRa configuration complete");
    log::info!("Frequency: {} MHz", FREQUENCY / 1_000_000);
    log::info!("Spreading Factor: {}", SPREADING_FACTOR);
    log::info!("Bandwidth: {} kHz", BANDWIDTH / 1_000);
    log::info!("Coding Rate: 4/{}", CODING_RATE);
    
    // Enter receive mode
    lora.start_receive()?;
    log::info!("LoRa module in receive mode");
    
    loop {
        // Check if packet is available
        if lora.is_packet_available()? {
            let packet = lora.read_packet()?;
            let packet_len = packet.len();
            
            if packet_len > 0 {
                // Convert to string if possible
                match std::str::from_utf8(&packet) {
                    Ok(message) => {
                        log::info!("Received LoRa message: {}", message);
                    }
                    Err(_) => {
                        log::info!("Received binary data: {:?}", packet);
                    }
                }
                
                // Process packet
                // ...
            }
        }
        
        // Sleep to prevent CPU hogging
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
}
```

### Lab Exercise
- Configure and test LoRa communication range
- Experiment with different spreading factors and bandwidths
- Measure power consumption at different settings
- Implement a simple point-to-point communication test

---

## Lecture 5.2: LoRa Communication Implementation

### Objectives
- Implement robust LoRa communication protocols
- Create reliable messaging systems over LoRa
- Handle transmission errors and retries

### Topics
1. **Packet Structure Design**
   - Header design
   - Addressing schemes
   - Payload formats
   - Error detection

2. **Reliable Communication**
   - Acknowledgment mechanisms
   - Retransmission strategies
   - Sequence numbering
   - Timeout handling

3. **Power Efficiency**
   - Duty cycle management
   - Sleep strategies
   - Adaptive power control
   - Listen-before-talk techniques

4. **Multi-Node Communication**
   - Broadcasting and multicasting
   - Node discovery
   - Channel access methods
   - Collision avoidance

### Code Example: LoRa Message Protocol
```rust
use anyhow::{anyhow, Result};
use crc::{Crc, CRC_16_IBM_SDLC};
use esp_idf_hal::delay::Ets;
use esp_idf_hal::peripherals::Peripherals;
use sx127x_lora::LoRa;
use std::time::{Duration, Instant};

// CRC calculator for message integrity
const X25: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_SDLC);

// Message types
const MSG_TYPE_DATA: u8 = 0x01;
const MSG_TYPE_ACK: u8 = 0x02;
const MSG_TYPE_PING: u8 = 0x03;
const MSG_TYPE_PONG: u8 = 0x04;

// Protocol constants
const MAX_PAYLOAD_SIZE: usize = 128;
const MAX_RETRIES: u8 = 3;
const ACK_TIMEOUT_MS: u64 = 1000;
const MSG_HEADER_SIZE: usize = 8; // Source + Dest + Type + SeqNum + Length

struct LoRaMessage {
    source_addr: u8,
    dest_addr: u8,
    msg_type: u8,
    seq_num: u8,
    payload: Vec<u8>,
}

impl LoRaMessage {
    fn new(source: u8, dest: u8, msg_type: u8, seq_num: u8, payload: &[u8]) -> Self {
        Self {
            source_addr: source,
            dest_addr: dest,
            msg_type,
            seq_num,
            payload: payload.to_vec(),
        }
    }
    
    fn serialize(&self) -> Vec<u8> {
        let mut data = Vec::with_capacity(MSG_HEADER_SIZE + self.payload.len() + 2); // +2 for CRC
        
        // Header
        data.push(self.source_addr);
        data.push(self.dest_addr);
        data.push(self.msg_type);
        data.push(self.seq_num);
        data.push(self.payload.len() as u8);
        
        // Payload
        data.extend_from_slice(&self.payload);
        
        // Calculate CRC
        let crc = X25.checksum(&data);
        data.push((crc >> 8) as u8);  // CRC high byte
        data.push(crc as u8);         // CRC low byte
        
        data
    }
    
    fn deserialize(data: &[u8]) -> Result<Self> {
        if data.len() < MSG_HEADER_SIZE + 2 {  // +2 for CRC
            return Err(anyhow!("Message too short"));
        }
        
        // Extract CRC
        let msg_len = data.len();
        let received_crc = ((data[msg_len-2] as u16) << 8) | (data[msg_len-1] as u16);
        
        // Verify CRC
        let calculated_crc = X25.checksum(&data[0..msg_len-2]);
        if calculated_crc != received_crc {
            return Err(anyhow!("CRC mismatch"));
        }
        
        // Extract header fields
        let source_addr = data[0];
        let dest_addr = data[1];
        let msg_type = data[2];
        let seq_num = data[3];
        let payload_len = data[4] as usize;
        
        // Verify payload length
        if payload_len != msg_len - MSG_HEADER_SIZE - 2 {
            return Err(anyhow!("Invalid payload length"));
        }
        
        // Extract payload
        let payload = data[5..5+payload_len].to_vec();
        
        Ok(Self {
            source_addr,
            dest_addr,
            msg_type,
            seq_num,
            payload,
        })
    }
}

struct LoRaNode {
    lora: LoRa,
    address: u8,
    seq_counter: u8,
}

impl LoRaNode {
    fn new(lora: LoRa, address: u8) -> Self {
        Self {
            lora,
            address,
            seq_counter: 0,
        }
    }
    
    fn send_message(&mut self, dest: u8, msg_type: u8, payload: &[u8]) -> Result<bool> {
        if payload.len() > MAX_PAYLOAD_SIZE {
            return Err(anyhow!("Payload too large"));
        }
        
        let seq_num = self.seq_counter;
        self.seq_counter = self.seq_counter.wrapping_add(1);
        
        let message = LoRaMessage::new(self.address, dest, msg_type, seq_num, payload);
        let packet = message.serialize();
        
        // Use different strategies based on message type
        match msg_type {
            MSG_TYPE_DATA => {
                // For data messages, use reliable delivery with ACK
                self.send_reliable(&packet)
            }
            _ => {
                // For other message types, just send once
                self.lora.transmit(&packet)?;
                Ok(true)
            }
        }
    }
    
    fn send_reliable(&mut self, packet: &[u8]) -> Result<bool> {
        let mut retries = 0;
        
        while retries < MAX_RETRIES {
            log::info!("Sending packet (attempt {})", retries + 1);
            self.lora.transmit(&packet)?;
            
            // Switch to receive mode and wait for ACK
            self.lora.start_receive()?;
            
            let start_time = Instant::now();
            let mut ack_received = false;
            
            while start_time.elapsed() < Duration::from_millis(ACK_TIMEOUT_MS) {
                if self.lora.is_packet_available()? {
                    let rx_packet = self.lora.read_packet()?;
                    
                    // Try to parse as message
                    if let Ok(message) = LoRaMessage::deserialize(&rx_packet) {
                        // Check if it's an ACK for our message
                        if message.msg_type == MSG_TYPE_ACK && 
                           message.dest_addr == self.address &&
                           message.payload.len() >= 1 && 
                           message.payload[0] == packet[3] {  // Check sequence number
                            ack_received = true;
                            break;
                        }
                    }
                }
                
                // Small delay to prevent CPU hogging
                Ets::delay_ms(10);
            }
            
            if ack_received {
                log::info!("ACK received, message delivered successfully");
                return Ok(true);
            }
            
            log::warn!("ACK timeout, retrying...");
            retries += 1;
        }
        
        log::error!("Failed to deliver message after {} attempts", MAX_RETRIES);
        Ok(false)
    }
    
    fn process_incoming_messages(&mut self) -> Result<()> {
        if self.lora.is_packet_available()? {
            let packet = self.lora.read_packet()?;
            
            match LoRaMessage::deserialize(&packet) {
                Ok(message) => {
                    // Check if message is for us or broadcast
                    if message.dest_addr == self.address || message.dest_addr == 0xFF {
                        match message.msg_type {
                            MSG_TYPE_DATA => {
                                log::info!("Received data message from {}: {:?}", 
                                    message.source_addr, message.payload);
                                
                                // Send ACK
                                let ack_payload = vec![message.seq_num];
                                let ack = LoRaMessage::new(
                                    self.address, 
                                    message.source_addr, 
                                    MSG_TYPE_ACK, 
                                    self.seq_counter, 
                                    &ack_payload
                                );
                                self.seq_counter = self.seq_counter.wrapping_add(1);
                                
                                let ack_packet = ack.serialize();
                                self.lora.transmit(&ack_packet)?;
                                
                                // Process the message
                                // ...
                            },
                            MSG_TYPE_PING => {
                                log::info!("Received ping from {}", message.source_addr);
                                
                                // Reply with pong
                                let pong = LoRaMessage::new(
                                    self.address, 
                                    message.source_addr, 
                                    MSG_TYPE_PONG, 
                                    self.seq_counter, 
                                    &[]
                                );
                                self.seq_counter = self.seq_counter.wrapping_add(1);
                                
                                let pong_packet = pong.serialize();
                                self.lora.transmit(&pong_packet)?;
                            },
                            MSG_TYPE_ACK => {
                                // ACKs are handled in the send_reliable method
                            },
                            _ => {
                                log::warn!("Unknown message type: {}", message.msg_type);
                            }
                        }
                    }
                },
                Err(e) => {
                    log::warn!("Failed to parse message: {}", e);
                }
            }
        }
        
        Ok(())
    }
}

fn main() -> anyhow::Result<()> {
    // Initialize ESP-IDF and LoRa hardware
    // (Same as in previous example)
    
    // Create LoRa node with address 0x01
    let mut node = LoRaNode::new(lora, 0x01);
    
    // Main loop
    loop {
        // Process incoming messages
        node.process_incoming_messages()?;
        
        // Every 10 seconds, send a ping to address 0x02
        static mut LAST_PING: Option<Instant> = None;
        
        unsafe {
            let now = Instant::now();
            
            if LAST_PING.is_none() || now.duration_since(LAST_PING.unwrap()) > Duration::from_secs(10) {
                log::info!("Sending ping to node 0x02");
                node.send_message(0x02, MSG_TYPE_PING, &[])?;
                LAST_PING = Some(now);
            }
        }
        
        // Small delay to prevent CPU hogging
        Ets::delay_ms(100);
    }
}
```

### Lab Exercise
- Implement a reliable message protocol over LoRa
- Create a multi-node network with addressing
- Test communication reliability at different distances
- Measure and optimize power consumption during communication

---

## Lecture 5.3: LoRa Network Topologies

### Objectives
- Design different LoRa network architectures
- Implement star, mesh, and hybrid topologies
- Optimize for coverage, reliability, and power consumption

### Topics
1. **Star Network Topology**
   - Central coordinator design
   - End-node implementation
   - Registration and discovery
   - Pros and cons for IoT applications

2. **Mesh Network Routing**
   - Routing algorithms for LoRa
   - Path discovery techniques
   - Message forwarding strategies
   - Network resilience approaches

3. **Gateway and Backhaul Integration**
   - LoRa to WiFi/Ethernet bridging
   - Cloud connectivity options
   - Store-and-forward mechanisms
   - Synchronization strategies

4. **Topology Selection Criteria**
   - Application requirements analysis
   - Coverage vs. power consumption
   - Latency considerations
   - Scalability factors

### Code Example: Simple LoRa Star Network
```rust
use anyhow::Result;
use esp_idf_hal::prelude::*;
use esp_idf_svc::timer::EspTimerService;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

// Network constants
const COORDINATOR_ADDRESS: u8 = 0x00;
const BROADCAST_ADDRESS: u8 = 0xFF;
const NODE_TIMEOUT_SECS: u64 = 300; // 5 minutes
const BEACON_INTERVAL_SECS: u64 = 60; // 1 minute

// Message types
const MSG_TYPE_REGISTER: u8 = 0x10;
const MSG_TYPE_REG_ACK: u8 = 0x11;
const MSG_TYPE_DATA: u8 = 0x20;
const MSG_TYPE_DATA_ACK: u8 = 0x21;
const MSG_TYPE_BEACON: u8 = 0x30;
const MSG_TYPE_STATUS: u8 = 0x40;

// Node information
struct NodeInfo {
    address: u8,
    last_seen: Instant,
    rssi: i32,
    battery_level: u8,
}

// Coordinator implementation
struct LoRaCoordinator {
    node: LoRaNode,
    registered_nodes: HashMap<u8, NodeInfo>,
    next_address: u8,
}

impl LoRaCoordinator {
    fn new(lora: LoRa) -> Self {
        Self {
            node: LoRaNode::new(lora, COORDINATOR_ADDRESS),
            registered_nodes: HashMap::new(),
            next_address: 0x01, // Start assigning from 0x01
        }
    }
    
    fn run(&mut self) -> Result<()> {
        log::info!("Starting LoRa coordinator");
        
        // Set up periodic beacon timer
        let timer_service = EspTimerService::new()?;
        let node_ptr = Arc::new(Mutex::new(&mut self.node));
        let node_clone = Arc::clone(&node_ptr);
        
        let _beacon_timer = timer_service.timer(move || {
            if let Ok(mut node) = node_clone.lock() {
                // Send network beacon
                log::info!("Sending network beacon");
                let _ = node.send_message(BROADCAST_ADDRESS, MSG_TYPE_BEACON, &[]);
            }
        })?;
        
        _beacon_timer.every(Duration::from_secs(BEACON_INTERVAL_SECS))?;
        
        // Main loop
        loop {
            // Process incoming messages
            if self.node.lora.is_packet_available()? {
                let packet = self.node.lora.read_packet()?;
                let rssi = self.node.lora.get_rssi()?;
                
                if let Ok(message) = LoRaMessage::deserialize(&packet) {
                    self.process_message(message, rssi)?;
                }
            }
            
            // Check for node timeouts
            self.check_node_timeouts();
            
            // Small delay to prevent CPU hogging
            Ets::delay_ms(100);
        }
    }
    
    fn process_message(&mut self, message: LoRaMessage, rssi: i32) -> Result<()> {
        // Update last seen time for existing nodes
        if message.source_addr != COORDINATOR_ADDRESS && 
           message.source_addr != BROADCAST_ADDRESS &&
           self.registered_nodes.contains_key(&message.source_addr) {
            
            let node = self.registered_nodes.get_mut(&message.source_addr).unwrap();
            node.last_seen = Instant::now();
            node.rssi = rssi;
        }
        
        // Process message based on type
        match message.msg_type {
            MSG_TYPE_REGISTER => {
                log::info!("Registration request from node");
                
                // Assign new address
                let new_address = self.next_address;
                self.next_address = self.next_address.wrapping_add(1);
                if self.next_address == 0 {
                    self.next_address = 1; // Skip address 0 (coordinator)
                }
                
                // Register the node
                self.registered_nodes.insert(new_address, NodeInfo {
                    address: new_address,
                    last_seen: Instant::now(),
                    rssi,
                    battery_level: if message.payload.len() > 0 { message.payload[0] } else { 0 },
                });
                
                log::info!("Registered new node with address: {:#04x}", new_address);
                
                // Send registration ACK with assigned address
                let payload = vec![new_address];
                self.node.send_message(BROADCAST_ADDRESS, MSG_TYPE_REG_ACK, &payload)?;
            },
            
            MSG_TYPE_DATA => {
                if self.registered_nodes.contains_key(&message.source_addr) {
                    log::info!("Data from node {:#04x}: {:?}", message.source_addr, message.payload);
                    
                    // Process data
                    // ...
                    
                    // Send ACK
                    let ack_payload = vec![message.seq_num];
                    self.node.send_message(message.source_addr, MSG_TYPE_DATA_ACK, &ack_payload)?;
                } else {
                    log::warn!("Data from unregistered node: {:#04x}", message.source_addr);
                }
            },
            
            MSG_TYPE_STATUS => {
                if message.source_addr != BROADCAST_ADDRESS && 
                   self.registered_nodes.contains_key(&message.source_addr) {
                    
                    // Update node status
                    if message.payload.len() > 0 {
                        let node = self.registered_nodes.get_mut(&message.source_addr).unwrap();
                        node.battery_level = message.payload[0];
                    }
                    
                    log::info!("Status update from node {:#04x}", message.source_addr);
                }
            },
            
            _ => {
                log::debug!("Received message type {:#04x} from {:#04x}", 
                    message.msg_type, message.source_addr);
            }
        }
        
        Ok(())
    }
    
    fn check_node_timeouts(&mut self) {
        let now = Instant::now();
        let mut expired_nodes = Vec::new();
        
        // Find expired nodes
        for (&addr, node) in &self.registered_nodes {
            if now.duration_since(node.last_seen) > Duration::from_secs(NODE_TIMEOUT_SECS) {
                expired_nodes.push(addr);
            }
        }
        
        // Remove expired nodes
        for addr in expired_nodes {
            log::warn!("Node {:#04x} timed out, removing", addr);
            self.registered_nodes.remove(&addr);
        }
    }
    
    fn print_network_status(&self) {
        log::info!("Network status:");
        log::info!("Coordinator address: {:#04x}", COORDINATOR_ADDRESS);
        log::info!("Registered nodes: {}", self.registered_nodes.len());
        
        for (addr, node) in &self.registered_nodes {
            log::info!("Node {:#04x}: RSSI = {} dBm, Battery = {}%, Last seen: {:?} ago",
                addr,
                node.rssi,
                node.battery_level,
                Instant::now().duration_since(node.last_seen)
            );
        }
    }
}

// End node implementation
struct LoRaEndNode {
    node: LoRaNode,
    coordinator_address: u8,
    registered: bool,
}

impl LoRaEndNode {
    fn new(lora: LoRa) -> Self {
        // Start with temporary address
        Self {
            node: LoRaNode::new(lora, BROADCAST_ADDRESS),
            coordinator_address: COORDINATOR_ADDRESS,
            registered: false,
        }
    }
    
    fn run(&mut self) -> Result<()> {
        log::info!("Starting LoRa end node");
        
        // Register with coordinator
        self.register()?;
        
        // Set up status update timer
        let timer_service = EspTimerService::new()?;
        let node_ptr = Arc::new(Mutex::new(&mut self.node));
        let node_clone = Arc::clone(&node_ptr);
        let coordinator_addr = self.coordinator_address;
        
        let _status_timer = timer_service.timer(move || {
            if let Ok(mut node) = node_clone.lock() {
                // Send status update (with battery level)
                let battery_level = read_battery_level();
                let _ = node.send_message(coordinator_addr, MSG_TYPE_STATUS, &[battery_level]);
            }
        })?;
        
        _status_timer.every(Duration::from_secs(120))?; // Every 2 minutes
        
        // Main loop
        loop {
            // Process incoming messages
            if self.node.lora.is_packet_available()? {
                let packet = self.node.lora.read_packet()?;
                
                if let Ok(message) = LoRaMessage::deserialize(&packet) {
                    self.process_message(message)?;
                }
            }
            
            // Perform sensor readings periodically
            static mut LAST_READING: Option<Instant> = None;
            
            unsafe {
                let now = Instant::now();
                
                if LAST_READING.is_none() || now.duration_since(LAST_READING.unwrap()) > Duration::from_secs(300) {
                    if self.registered {
                        // Read sensor data
                        let sensor_data = read_sensor_data();
                        
                        // Send to coordinator
                        log::info!("Sending sensor data to coordinator");
                        self.node.send_message(self.coordinator_address, MSG_TYPE_DATA, &sensor_data)?;
                    }
                    
                    LAST_READING = Some(now);
                }
            }
            
            // Energy-saving delay
            Ets::delay_ms(100);
        }
    }
    
    fn register(&mut self) -> Result<()> {
        let mut attempts = 0;
        
        while !self.registered && attempts < 5 {
            log::info!("Attempting to register with coordinator (attempt {})", attempts + 1);
            
            // Get battery level
            let battery_level = read_battery_level();
            
            // Send registration request
            self.node.send_message(COORDINATOR_ADDRESS, MSG_TYPE_REGISTER, &[battery_level])?;
            
            // Wait for response
            let start_time = Instant::now();
            
            while start_time.elapsed() < Duration::from_secs(5) && !self.registered {
                if self.node.lora.is_packet_available()? {
                    let packet = self.node.lora.read_packet()?;
                    
                    if let Ok(message) = LoRaMessage::deserialize(&packet) {
                        if message.msg_type == MSG_TYPE_REG_ACK && 
                           message.source_addr == COORDINATOR_ADDRESS && 
                           message.payload.len() > 0 {
                            
                            // Get assigned address
                            let assigned_address = message.payload[0];
                            self.node.address = assigned_address;
                            self.registered = true;
                            
                            log::info!("Registered successfully with address: {:#04x}", assigned_address);
                            return Ok(());
                        }
                    }
                }
                
                Ets::delay_ms(100);
            }
            
            attempts += 1;
        }
        
        if !self.registered {
            log::error!("Failed to register with coordinator");
            return Err(anyhow!("Registration failed"));
        }
        
        Ok(())
    }
    
    fn process_message(&mut self, message: LoRaMessage) -> Result<()> {
        // Only process messages for us or broadcasts
        if message.dest_addr != self.node.address && message.dest_addr != BROADCAST_ADDRESS {
            return Ok(());
        }
        
        match message.msg_type {
            MSG_TYPE_BEACON => {
                log::debug!("Received network beacon");
                // Update coordinator timestamp
                // ...
            },
            
            MSG_TYPE_DATA_ACK => {
                log::debug!("Data acknowledgment received");
                // Verify sequence number
                // ...
            },
            
            _ => {
                log::debug!("Received message type {:#04x} from {:#04x}", 
                    message.msg_type, message.source_addr);
            }
        }
        
        Ok(())
    }
}

// Utility functions
fn read_battery_level() -> u8 {
    // In a real application, read from ADC
    // For example purposes, return a fixed value
    85 // 85%
}

fn read_sensor_data() -> Vec<u8> {
    // In a real application, read from actual sensors
    // For example purposes, return some dummy data
    vec![0x01, 0x02, 0x22, 0x68, 0x3A] // Some sensor data
}
```

### Lab Exercise
- Implement a star network with one coordinator and multiple nodes
- Create a simple routing protocol for mesh networking
- Test network coverage and reliability in different environments
- Measure and optimize power consumption in different topologies

---

## Lecture 5.4: Project - Remote Monitoring System

### Objectives
- Create a complete remote monitoring system using LoRa
- Implement reliable long-range communication
- Design for power efficiency and robust operation

### Topics
1. **System Architecture**
   - Node types and responsibilities
   - Network topology selection
   - Power and range requirements
   - Data collection strategy

2. **Sensor Integration**
   - Environmental monitoring sensors
   - Data acquisition and aggregation
   - Calibration and filtering
   - Storage and transmission strategy

3. **Network Protocol Design**
   - Message format specification
   - Addressing and routing approach
   - Error handling and recovery
   - Security considerations

4. **Gateway Implementation**
   - Data aggregation and storage
   - WiFi/Internet connectivity
   - Cloud service integration
   - Remote management capabilities

### Project Specification: Environmental Monitoring Network

**Requirements**
1. Implement a LoRa network with:
   - One central gateway node (with WiFi)
   - Multiple sensor nodes (battery-powered)
   - Long-range communication (1km+ target)
   
2. Sensor capabilities:
   - Temperature and humidity monitoring
   - Optional additional sensors (light, soil moisture, etc.)
   - Configurable sampling rates
   - Battery level reporting
   
3. Gateway features:
   - Data aggregation and local storage
   - Web interface for data visualization
   - Alarm notifications for out-of-range values
   - Remote configuration capability
   
4. Power management:
   - Low-power sleep modes
   - Adaptive transmission power
   - Battery life optimization
   - Energy harvesting support (optional)

**Deliverables**
1. Complete Rust projects for both node types
2. Network protocol documentation
3. Hardware setup instructions
4. Web interface for gateway
5. Performance and range test report

---

# Module 6: Advanced Topics

## Lecture 6.1: Rust's Concurrency Model on ESP32

### Objectives
- Understand Rust's concurrency primitives in embedded contexts
- Learn to use FreeRTOS tasks with Rust safely
- Implement proper synchronization and communication between tasks

### Topics
1. **Rust Concurrency Basics**
   - Thread safety in Rust
   - Send and Sync traits
   - Arc and Mutex patterns
   - ESP32-specific considerations

2. **FreeRTOS Task Management**
   - Task creation and lifecycle
   - Priority scheduling
   - Stack allocation
   - Rust wrappers for FreeRTOS functions

3. **Message Passing Patterns**
   - Queues and channels
   - Event groups
   - Notification mechanisms
   - Zero-copy communication

4. **Synchronization Primitives**
   - Mutexes and semaphores
   - Critical sections
   - Atomic operations
   - Deadlock prevention

### Code Example: Task-Based Sensor Monitoring
```rust
use anyhow::Result;
use esp_idf_hal::prelude::*;
use esp_idf_svc::task::EspTaskBuilder;
use std::sync::{Arc, Mutex};
use std::time::Duration;

// Shared sensor data
struct SensorData {
    temperature: f32,
    humidity: u8,
    pressure: u32,
    updated_at: u64,
}

// Task parameters
const SENSOR_TASK_STACK_SIZE: usize = 4096;
const DISPLAY_TASK_STACK_SIZE: usize = 4096;
const NETWORK_TASK_STACK_SIZE: usize = 8192;

const SENSOR_TASK_PRIORITY: u8 = 5;
const DISPLAY_TASK_PRIORITY: u8 = 4;
const NETWORK_TASK_PRIORITY: u8 = 3;

fn main() -> Result<()> {
    // Initialize ESP-IDF
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    
    log::info!("Starting multitasking example");
    
    // Create shared sensor data
    let sensor_data = Arc::new(Mutex::new(SensorData {
        temperature: 0.0,
        humidity: 0,
        pressure: 0,
        updated_at: 0,
    }));
    
    // Create sensor reading task
    let sensor_data_clone = Arc::clone(&sensor_data);
    let _sensor_task = EspTaskBuilder::new()
        .stack_size(SENSOR_TASK_STACK_SIZE)
        .priority(SENSOR_TASK_PRIORITY)
        .name("sensor_task")
        .spawn(move || {
            sensor_task(sensor_data_clone)
        })?;
    
    // Create display update task
    let sensor_data_clone = Arc::clone(&sensor_data);
    let _display_task = EspTaskBuilder::new()
        .stack_size(DISPLAY_TASK_STACK_SIZE)
        .priority(DISPLAY_TASK_PRIORITY)
        .name("display_task")
        .spawn(move || {
            display_task(sensor_data_clone)
        })?;
    
    // Create network communication task
    let sensor_data_clone = Arc::clone(&sensor_data);
    let _network_task = EspTaskBuilder::new()
        .stack_size(NETWORK_TASK_STACK_SIZE)
        .priority(NETWORK_TASK_PRIORITY)
        .name("network_task")
        .spawn(move || {
            network_task(sensor_data_clone)
        })?;
    
    log::info!("All tasks started");
    
    // Main task can do other work or just sleep
    loop {
        std::thread::sleep(Duration::from_secs(60));
    }
}

fn sensor_task(sensor_data: Arc<Mutex<SensorData>>) {
    log::info!("Sensor task started");
    
    // In a real application, we would initialize sensor hardware here
    // let mut bme280 = setup_bme280().expect("Failed to initialize BME280");
    
    loop {
        // Read sensor values (simulated here)
        let temperature = 22.5 + (esp_idf_sys::esp_random() % 100) as f32 / 100.0;
        let humidity = 50 + (esp_idf_sys::esp_random() % 20) as u8;
        let pressure = 101300 + (esp_idf_sys::esp_random() % 200) as u32;
        
        // Get current timestamp
        let now = esp_idf_sys::esp_timer_get_time() / 1000; // milliseconds
        
        // Update shared state
        {
            // Scope for mutex lock to ensure it's released promptly
            log::debug!("Acquiring sensor data mutex from sensor task");
            let mut data = sensor_data.lock().unwrap();
            data.temperature = temperature;
            data.humidity = humidity;
            data.pressure = pressure;
            data.updated_at = now;
            log::debug!("Released sensor data mutex from sensor task");
        }
        
        log::info!("Sensor readings: {:.1}°C, {}%, {} Pa", temperature, humidity, pressure);
        
        // Sleep for sensor sampling interval
        std::thread::sleep(Duration::from_secs(2));
    }
}

fn display_task(sensor_data: Arc<Mutex<SensorData>>) {
    log::info!("Display task started");
    
    // In a real application, we would initialize display hardware here
    // let mut display = setup_display().expect("Failed to initialize display");
    
    loop {
        // Get sensor data for display
        let (temp, humidity, pressure, timestamp);
        
        {
            log::debug!("Acquiring sensor data mutex from display task");
            let data = sensor_data.lock().unwrap();
            temp = data.temperature;
            humidity = data.humidity;
            pressure = data.pressure;
            timestamp = data.updated_at;
            log::debug!("Released sensor data mutex from display task");
        }
        
        // Update display (simulated here)
        log::info!("Updating display with: {:.1}°C, {}%, {} Pa (updated at: {})",
            temp, humidity, pressure, timestamp);
        
        // In a real application, we would update the display
        // display.clear();
        // display.print_line(0, &format!("Temp: {:.1}°C", temp));
        // display.print_line(1, &format!("Humidity: {}%", humidity));
        // display.print_line(2, &format!("Pressure: {} hPa", pressure / 100));
        // display.update();
        
        // Sleep for display update interval
        std::thread::sleep(Duration::from_secs(1));
    }
}

fn network_task(sensor_data: Arc<Mutex<SensorData>>) {
    log::info!("Network task started");
    
    // In a real application, we would initialize networking here
    // let mut wifi = setup_wifi().expect("Failed to initialize WiFi");
    // let mut api_client = setup_api_client().expect("Failed to initialize API client");
    
    loop {
        // Get sensor data for network transmission
        let (temp, humidity, pressure, timestamp);
        
        {
            log::debug!("Acquiring sensor data mutex from network task");
            let data = sensor_data.lock().unwrap();
            temp = data.temperature;
            humidity = data.humidity;
            pressure = data.pressure;
            timestamp = data.updated_at;
            log::debug!("Released sensor data mutex from network task");
        }
        
        // Simulate network transmission
        log::info!("Sending data to server: {:.1}°C, {}%, {} Pa (updated at: {})",
            temp, humidity, pressure, timestamp);
        
        // In a real application, we would send data to a server
        // let payload = json!({
        //     "temperature": temp,
        //     "humidity": humidity,
        //     "pressure": pressure,
        //     "timestamp": timestamp
        // });
        // api_client.post_data(&payload).unwrap_or_else(|e| {
        //     log::error!("Failed to send data: {}", e);
        // });
        
        // Sleep for network transmission interval (longer than sensing interval)
        std::thread::sleep(Duration::from_secs(30));
    }
}
```

### Lab Exercise
- Implement a multitasking application with sensor reading, display, and networking
- Create a task communication system using message queues
- Implement proper synchronization for shared resources
- Measure and optimize task performance and memory usage

---

## Lecture 6.2: RTOS Integration in Rust

### Objectives
- Master FreeRTOS features through Rust abstractions
- Implement efficient task communication patterns
- Use advanced scheduler features for real-time applications

### Topics
1. **FreeRTOS Scheduler Optimization**
   - Task priorities and scheduling
   - Idle task hooks
   - Tick rate configuration
   - Time slicing options

2. **Advanced Queue Usage**
   - Queue sets
   - Message buffers
   - Stream buffers
   - Direct-to-task notifications

3. **Timer and Event Management**
   - Software timers
   - Event groups
   - Deferred interrupt processing
   - Task notifications

4. **Memory Management**
   - Static vs. dynamic allocation
   - Memory protection
   - Stack overflow detection
   - Heap fragmentation prevention

### Code Example: Advanced RTOS Patterns
```rust
use anyhow::{anyhow, Result};
use esp_idf_hal::delay::Ets;
use esp_idf_hal::prelude::*;
use esp_idf_svc::systime::EspSystemTime;
use esp_idf_svc::task::EspTaskBuilder;
use esp_idf_svc::timer::{EspTimer, EspTimerService};
use esp_idf_svc::eventloop::{EspSystemEventLoop, EventLoop};
use esp_idf_svc::notify::{EspNotify, Notification};
use std::sync::{Arc, Mutex};
use std::time::Duration;

// Event types for our system
#[derive(Debug, Copy, Clone, PartialEq)]
enum SystemEvent {
    SensorDataReady = 1,
    ButtonPressed = 2,
    AlarmTriggered = 3,
    NetworkConnected = 4,
    NetworkDisconnected = 5,
    LowBattery = 6,
}

// Notification flags for direct task communication
const NOTIFY_SENSOR_UPDATE: u32 = 0x01;
const NOTIFY_DISPLAY_UPDATE: u32 = 0x02;
const NOTIFY_ERROR_STATE: u32 = 0x04;
const NOTIFY_SHUTDOWN: u32 = 0x08;

// Queue for sensor data
#[derive(Debug, Copy, Clone)]
struct SensorMessage {
    sensor_id: u8,
    value: f32,
    timestamp: u64,
}

// System state
struct SystemState {
    running: bool,
    error_code: Option<u32>,
    last_update: u64,
    battery_level: u8,
}

fn main() -> Result<()> {
    // Initialize ESP-IDF
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    
    log::info!("Starting advanced RTOS example");
    
    // System time
    let systime = EspSystemTime {};
    
    // Create shared state
    let system_state = Arc::new(Mutex::new(SystemState {
        running: true,
        error_code: None,
        last_update: systime.now() as u64,
        battery_level: 100,
    }));
    
    // Create event loop for system events
    let system_events = Arc::new(EspEventLoop::<SystemEvent>::new()?);
    
    // Create message queue for sensor data
    let sensor_queue = Arc::new(Mutex::new(Vec::<SensorMessage>::with_capacity(32)));
    
    // Set up timer service
    let timer_service = EspTimerService::new()?;
    
    // Create periodic health check timer
    let system_state_clone = Arc::clone(&system_state);
    let system_events_clone = Arc::clone(&system_events);
    let _health_timer = timer_service.timer(move || {
        let mut state = system_state_clone.lock().unwrap();
        
        // Check battery level (simulated here)
        if state.battery_level > 0 {
            state.battery_level -= 1;
        }
        
        // If battery is low, fire an event
        if state.battery_level < 20 {
            log::warn!("Battery level low: {}%", state.battery_level);
            if let Err(e) = system_events_clone.post(&SystemEvent::LowBattery) {
                log::error!("Failed to post low battery event: {:?}", e);
            }
        }
        
        // Update last health check time
        state.last_update = systime.now() as u64;
    })?;
    
    _health_timer.every(Duration::from_secs(60))?;
    
    // Create sensor task with higher priority
    let sensor_queue_clone = Arc::clone(&sensor_queue);
    let system_events_clone = Arc::clone(&system_events);
    let _sensor_task = EspTaskBuilder::new()
        .stack_size(4096)
        .priority(10)  // Higher priority
        .name("sensor_task")
        .spawn(move || {
            sensor_task(sensor_queue_clone, system_events_clone)
        })?;
    
    // Create processing task with medium priority
    let sensor_queue_clone = Arc::clone(&sensor_queue);
    let system_events_clone = Arc::clone(&system_events);
    let system_state_clone = Arc::clone(&system_state);
    let processor_notify = Arc::new(EspNotify::new()?);
    let processor_notify_clone = Arc::clone(&processor_notify);
    
    let _process_task = EspTaskBuilder::new()
        .stack_size(8192)
        .priority(5)  // Medium priority
        .name("process_task")
        .spawn(move || {
            process_task(
                sensor_queue_clone, 
                system_events_clone, 
                system_state_clone,
                processor_notify_clone
            )
        })?;
    
    // Create display task with lower priority
    let system_events_clone = Arc::clone(&system_events);
    let system_state_clone = Arc::clone(&system_state);
    let _display_task = EspTaskBuilder::new()
        .stack_size(4096)
        .priority(3)  // Lower priority
        .name("display_task")
        .spawn(move || {
            display_task(system_events_clone, system_state_clone)
        })?;
    
    // Subscribe to system events in main task
    system_events.subscribe(|event| {
        match event {
            SystemEvent::AlarmTriggered => {
                log::warn!("Alarm triggered!");
                // Handle alarm condition
            },
            SystemEvent::NetworkConnected => {
                log::info!("Network connected");
                // Start network-dependent processes
            },
            SystemEvent::NetworkDisconnected => {
                log::warn!("Network disconnected");
                // Enter offline mode
            },
            _ => {
                log::debug!("Received system event: {:?}", event);
            }
        }
    })?;
    
    // Simulate some system events
    std::thread::spawn(move || {
        // Give tasks time to start
        Ets::delay_ms(2000);
        
        // Post some events
        if let Err(e) = system_events.post(&SystemEvent::NetworkConnected) {
            log::error!("Failed to post event: {:?}", e);
        }
        
        Ets::delay_ms(5000);
        
        // Notify the processor task directly
        if let Err(e) = processor_notify.notify(NOTIFY_SENSOR_UPDATE) {
            log::error!("Failed to notify processor task: {:?}", e);
        }
        
        Ets::delay_ms(10000);
        
        // Simulate alarm
        if let Err(e) = system_events.post(&SystemEvent::AlarmTriggered) {
            log::error!("Failed to post event: {:?}", e);
        }
    });
    
    // Main task stays alive
    loop {
        std::thread::sleep(Duration::from_secs(1));
        
        // Check if we should exit
        let running = {
            let state = system_state.lock().unwrap();
            state.running
        };
        
        if !running {
            log::info!("System shutdown requested, exiting main loop");
            break;
        }
    }
    
    Ok(())
}

fn sensor_task(
    sensor_queue: Arc<Mutex<Vec<SensorMessage>>>,
    system_events: Arc<EspEventLoop<SystemEvent>>
) {
    log::info!("Sensor task started");
    
    // Simulate multiple sensors
    let sensor_ids = [1, 2, 3];  // Temperature, humidity, pressure
    let mut counter = 0;
    let systime = EspSystemTime {};
    
    loop {
        // Simulate sensor readings
        for &sensor_id in &sensor_ids {
            let value = match sensor_id {
                1 => 22.0 + (counter % 10) as f32 / 10.0,  // Temperature
                2 => 50.0 + (counter % 20) as f32 / 2.0,   // Humidity
                3 => 1013.0 + (counter % 30) as f32 / 10.0, // Pressure
                _ => 0.0,
            };
            
            // Create sensor message
            let message = SensorMessage {
                sensor_id,
                value,
                timestamp: systime.now() as u64,
            };
            
            // Add to queue
            {
                let mut queue = sensor_queue.lock().unwrap();
                if queue.len() >= queue.capacity() {
                    // Queue full, remove oldest item
                    queue.remove(0);
                }
                queue.push(message);
                log::debug!("Added sensor data to queue: {:?}", message);
            }
        }
        
        // Post event to notify that new sensor data is available
        if let Err(e) = system_events.post(&SystemEvent::SensorDataReady) {
            log::error!("Failed to post sensor data event: {:?}", e);
        }
        
        // Increment counter
        counter += 1;
        
        // Sleep for sensor sampling interval
        std::thread::sleep(Duration::from_secs(2));
    }
}

fn process_task(
    sensor_queue: Arc<Mutex<Vec<SensorMessage>>>,
    system_events: Arc<EspEventLoop<SystemEvent>>,
    system_state: Arc<Mutex<SystemState>>,
    notify: Arc<EspNotify>
) {
    log::info!("Process task started");
    
    // Subscribe to relevant system events
    system_events.subscribe(|event| {
        if *event == SystemEvent::SensorDataReady {
            log::debug!("Process task notified of new sensor data");
        }
    }).expect("Failed to subscribe to events");
    
    loop {
        // Wait for notification or timeout
        match notify.wait(Duration::from_secs(5)) {
            Ok(flags) => {
                log::debug!("Process task received notification flags: {:#x}", flags);
                
                if flags & NOTIFY_SHUTDOWN != 0 {
                    log::info!("Process task received shutdown notification");
                    break;
                }
                
                if flags & NOTIFY_ERROR_STATE != 0 {
                    log::warn!("Process task entering error handling state");
                    // Handle error state
                    // ...
                }
                
                if flags & NOTIFY_SENSOR_UPDATE != 0 {
                    process_sensor_data(&sensor_queue);
                }
            },
            Err(esp_idf_svc::notify::Error::Timeout) => {
                // Timeout occurred, do background processing
                log::debug!("Process task timeout, doing background work");
                process_sensor_data(&sensor_queue);
            },
            Err(e) => {
                log::error!("Error in process task notification: {:?}", e);
            }
        }
        
        // Check system state
        let running = {
            let state = system_state.lock().unwrap();
            if let Some(error_code) = state.error_code {
                log::warn!("System error detected, code: {}", error_code);
            }
            state.running
        };
        
        if !running {
            log::info!("Process task detected system shutdown, exiting");
            break;
        }
    }
}

fn process_sensor_data(sensor_queue: &Arc<Mutex<Vec<SensorMessage>>>) {
    // Process all available sensor data
    let messages: Vec<SensorMessage> = {
        let mut queue = sensor_queue.lock().unwrap();
        let messages = queue.clone();
        queue.clear();
        messages
    };
    
    if !messages.is_empty() {
        log::info!("Processing {} sensor messages", messages.len());
        
        // Group by sensor ID
        let mut temp_readings = Vec::new();
        let mut humidity_readings = Vec::new();
        let mut pressure_readings = Vec::new();
        
        for msg in &messages {
            match msg.sensor_id {
                1 => temp_readings.push(msg.value),
                2 => humidity_readings.push(msg.value),
                3 => pressure_readings.push(msg.value),
                _ => {}
            }
        }
        
        // Calculate averages
        if !temp_readings.is_empty() {
            let avg_temp = temp_readings.iter().sum::<f32>() / temp_readings.len() as f32;
            log::info!("Average temperature: {:.1}°C", avg_temp);
        }
        
        if !humidity_readings.is_empty() {
            let avg_humidity = humidity_readings.iter().sum::<f32>() / humidity_readings.len() as f32;
            log::info!("Average humidity: {:.1}%", avg_humidity);
        }
        
        if !pressure_readings.is_empty() {
            let avg_pressure = pressure_readings.iter().sum::<f32>() / pressure_readings.len() as f32;
            log::info!("Average pressure: {:.1} hPa", avg_pressure);
        }
    }
}

fn display_task(
    system_events: Arc<EspEventLoop<SystemEvent>>,
    system_state: Arc<Mutex<SystemState>>
) {
    log::info!("Display task started");
    
    // Subscribe to relevant system events
    system_events.subscribe(|event| {
        match event {
            SystemEvent::SensorDataReady => {
                log::debug!("Display task updating for new sensor data");
                // Update display with new sensor data
            },
            SystemEvent::AlarmTriggered => {
                log::warn!("Display task showing alarm");
                // Show alarm on display
            },
            SystemEvent::LowBattery => {
                log::warn!("Display task showing low battery warning");
                // Show low battery warning
            },
            _ => {}
        }
    }).expect("Failed to subscribe to events");
    
    loop {
        // Simulate display updates
        log::debug!("Display task updating screen");
        
        // Get and display system state
        let (running, error_code, last_update, battery_level) = {
            let state = system_state.lock().unwrap();
            (state.running, state.error_code, state.last_update, state.battery_level)
        };
        
        log::info!("System state - Running: {}, Error: {:?}, Last Update: {}, Battery: {}%",
            running, error_code, last_update, battery_level);
        
        // Break if system is shutting down
        if !running {
            log::info!("Display task detected system shutdown, exiting");
            break;
        }
        
        // Sleep for display update interval
        std::thread::sleep(Duration::from_secs(2));
    }
}
```

### Lab Exercise
- Create a real-time application with multiple synchronized tasks
- Implement both event-based and direct notification communications
- Design a memory-efficient task hierarchy with appropriate priorities
- Measure and optimize system responsiveness under load

---

## Lecture 6.3: Power Management Techniques

### Objectives
- Implement effective power management strategies
- Maximize battery life in ESP32 applications
- Balance performance and power consumption

### Topics
1. **ESP32 Power Modes**
   - Active mode optimization
   - Light sleep mode
   - Deep sleep mode
   - Hibernation mode
   - Wake-up sources configuration

2. **Peripheral Power Management**
   - Power gating techniques
   - Clock gating strategies
   - Dynamic frequency scaling
   - Peripheral selective enabling

3. **Application-Level Optimization**
   - Duty cycling approach
   - Event-driven architecture
   - Idle task optimization
   - Flash and RAM power considerations

4. **Battery Management**
   - Battery monitoring techniques
   - Charging control (if applicable)
   - Discharge profile considerations
   - Power source switching

### Code Example: Advanced Power Management
```rust
use anyhow::Result;
use esp_idf_hal::adc::{AdcChannelDriver, AdcDriver, Atten11dB};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{Gpio34, Input, PinDriver};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_hal::prelude::*;
use esp_idf_hal::ulp;
use esp_idf_svc::nvs::{EspNvs, EspDefaultNvsPartition, NvsOpen};
use esp_idf_svc::timer::EspTimerService;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

// Sleep modes
enum SleepMode {
    Active,    // Normal operation
    Light,     // CPU sleeping, peripherals active
    Deep,      // Everything off except RTC
    Hibernate, // Only RTC timer active
}

// System power state
struct PowerState {
    mode: SleepMode,
    battery_voltage: f32,
    battery_percentage: u8,
    external_power: bool,
    last_active_time: u64,
    sleep_interval_ms: u64,
    activity_counter: u32,
}

// Wake-up reasons
#[derive(Debug, Clone, Copy, PartialEq)]
enum WakeReason {
    Timer,
    ExternalInterrupt,
    UlpProgram,
    Unknown,
}

// Global wake flag for interrupt handling
static WAKE_FLAG: AtomicBool = AtomicBool::new(false);

fn main() -> Result<()> {
    // Initialize ESP-IDF
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    
    log::info!("Starting power management example");
    
    // Initialize NVS for storing settings
    let nvs = EspDefaultNvsPartition::take()?;
    let mut storage = EspNvs::new(nvs, "power_mgmt", NvsOpen::ReadWrite)?;
    
    // Read sleep interval from NVS or use default
    let default_sleep_interval: u64 = 60000; // 60 seconds
    let sleep_interval = match storage.get_u64("sleep_interval") {
        Ok(Some(interval)) => interval,
        _ => {
            // Store default value if not found
            let _ = storage.set_u64("sleep_interval", default_sleep_interval);
            default_sleep_interval
        }
    };
    
    // Get wake reason
    let wake_reason = get_wake_reason();
    log::info!("Wake reason: {:?}", wake_reason);
    
    // Create power state
    let power_state = Arc::new(Mutex::new(PowerState {
        mode: SleepMode::Active,
        battery_voltage: 0.0,
        battery_percentage: 0,
        external_power: false,
        last_active_time: esp_idf_sys::esp_timer_get_time() as u64,
        sleep_interval_ms: sleep_interval,
        activity_counter: 0,
    }));
    
    // Initialize hardware
    let peripherals = Peripherals::take()?;
    
    // Configure ADC for battery monitoring
    let mut adc = AdcDriver::new(peripherals.adc1, &Default::default())?;
    let mut battery_pin: AdcChannelDriver<_, Atten11dB<_>> = 
        AdcChannelDriver::new(peripherals.pins.gpio34)?;
    
    // Configure wake button
    let mut wake_button = PinDriver::input(peripherals.pins.gpio0)?;
    wake_button.set_pull(esp_idf_hal::gpio::Pull::Up)?;
    
    // Configure wake-up sources for deep sleep
    configure_wakeup_sources()?;
    
    // Set up battery monitoring timer
    let power_state_clone = Arc::clone(&power_state);
    let timer_service = EspTimerService::new()?;
    let _battery_timer = timer_service.timer(move || {
        let mut state = power_state_clone.lock().unwrap();
        
        // Read battery voltage (simulated here)
        let raw_value = adc.read(&mut battery_pin).unwrap_or(0);
        
        // Convert to voltage (adjust these values based on your hardware)
        let voltage = (raw_value as f32) * 3.3 / 4095.0 * 2.0; // Assuming voltage divider
        
        // Update battery state
        state.battery_voltage = voltage;
        state.battery_percentage = battery_voltage_to_percentage(voltage);
        
        // Check for external power (simulated)
        state.external_power = voltage > 4.0;
        
        log::info!("Battery: {:.2}V ({}%), External power: {}", 
            voltage, 
            state.battery_percentage,
            if state.external_power { "Yes" } else { "No" });
    })?;
    
    _battery_timer.every(Duration::from_secs(30))?;
    
    // Main activity loop
    let mut consecutive_idle_periods = 0;
    let power_state_clone = Arc::clone(&power_state);
    
    loop {
        // Check for button press or other activity
        let activity_detected = WAKE_FLAG.swap(false, Ordering::Relaxed) ||
                                 wake_button.is_low()?;
        
        if activity_detected {
            log::info!("Activity detected, resetting idle counter");
            consecutive_idle_periods = 0;
            
            // Update power state
            let mut state = power_state_clone.lock().unwrap();
            state.last_active_time = esp_idf_sys::esp_timer_get_time() as u64;
            state.activity_counter += 1;
            state.mode = SleepMode::Active;
        } else {
            consecutive_idle_periods += 1;
        }
        
        // Do main application work here
        // ...
        
        // Check power state and decide on sleep strategy
        let (should_sleep, sleep_mode, sleep_duration) = {
            let state = power_state_clone.lock().unwrap();
            
            // If on external power, be more liberal with staying awake
            if state.external_power {
                if consecutive_idle_periods > 10 {
                    (true, SleepMode::Light, 1000) // Light sleep after 10 idle periods
                } else {
                    (false, SleepMode::Active, 0)
                }
            } else {
                // On battery, be more aggressive with power saving
                if consecutive_idle_periods > 30 {
                    (true, SleepMode::Deep, state.sleep_interval_ms) // Deep sleep after 30 idle periods
                } else if consecutive_idle_periods > 5 {
                    (true, SleepMode::Light, 1000) // Light sleep after 5 idle periods
                } else {
                    (false, SleepMode::Active, 0)
                }
            }
        };
        
        if should_sleep {
            match sleep_mode {
                SleepMode::Light => {
                    log::info!("Entering light sleep for {} ms", sleep_duration);
                    
                    // Update mode
                    power_state_clone.lock().unwrap().mode = SleepMode::Light;
                    
                    // Light sleep
                    esp_idf_hal::delay::FreeRtos::sleep_ms(sleep_duration as u32);
                },
                SleepMode::Deep => {
                    log::info!("Entering deep sleep for {} ms", sleep_duration);
                    
                    // Store any important state to RTC memory or NVS
                    let counter = power_state_clone.lock().unwrap().activity_counter;
                    let _ = storage.set_u32("activity_counter", counter);
                    let _ = storage.commit();
                    
                    // Ensure logs are flushed
                    log::logger().flush();
                    
                    // Set wakeup timer
                    unsafe {
                        esp_idf_sys::esp_sleep_enable_timer_wakeup(sleep_duration * 1000);
                        
                        // Enter deep sleep (doesn't return)
                        esp_idf_sys::esp_deep_sleep_start();
                    }
                    
                    // Shouldn't reach here
                    unreachable!();
                },
                SleepMode::Hibernate => {
                    log::info!("Entering hibernation mode");
                    
                    // Store any important state
                    let _ = storage.commit();
                    
                    // Ensure logs are flushed
                    log::logger().flush();
                    
                    // Configure hibernation mode
                    unsafe {
                        // Disable all wakeup sources except RTC timer
                        esp_idf_sys::esp_sleep_disable_wakeup_source(
                            esp_idf_sys::esp_sleep_source_t_ESP_SLEEP_WAKEUP_ALL as u32);
                        esp_idf_sys::esp_sleep_enable_timer_wakeup(sleep_duration * 1000);
                        
                        // Enter deep sleep with maximum power savings
                        esp_idf_sys::esp_sleep_pd_config(
                            esp_idf_sys::esp_sleep_pd_domain_t_ESP_PD_DOMAIN_RTC_PERIPH,
                            esp_idf_sys::esp_sleep_pd_option_t_ESP_PD_OPTION_OFF);
                        esp_idf_sys::esp_sleep_pd_config(
                            esp_idf_sys::esp_sleep_pd_domain_t_ESP_PD_DOMAIN_RTC_SLOW_MEM,
                            esp_idf_sys::esp_sleep_pd_option_t_ESP_PD_OPTION_OFF);
                        esp_idf_sys::esp_sleep_pd_config(
                            esp_idf_sys::esp_sleep_pd_domain_t_ESP_PD_DOMAIN_RTC_FAST_MEM,
                            esp_idf_sys::esp_sleep_pd_option_t_ESP_PD_OPTION_OFF);
                            
                        esp_idf_sys::esp_deep_sleep_start();
                    }
                    
                    // Shouldn't reach here
                    unreachable!();
                },
                SleepMode::Active => {
                    // Just a short delay
                    FreeRtos::delay_ms(100);
                }
            }
        } else {
            // Short delay to prevent CPU hogging in active mode
            FreeRtos::delay_ms(100);
        }
    }
}

fn get_wake_reason() -> WakeReason {
    unsafe {
        let reason = esp_idf_sys::esp_sleep_get_wakeup_cause();
        match reason {
            esp_idf_sys::esp_sleep_source_t_ESP_SLEEP_WAKEUP_TIMER => WakeReason::Timer,
            esp_idf_sys::esp_sleep_source_t_ESP_SLEEP_WAKEUP_EXT0 |
            esp_idf_sys::esp_sleep_source_t_ESP_SLEEP_WAKEUP_EXT1 => WakeReason::ExternalInterrupt,
            esp_idf_sys::esp_sleep_source_t_ESP_SLEEP_WAKEUP_ULP => WakeReason::UlpProgram,
            _ => WakeReason::Unknown,
        }
    }
}

fn configure_wakeup_sources() -> Result<()> {
    unsafe {
        // Configure EXT0 (single GPIO) wakeup from deep sleep
        // This configures GPIO0 (typically the BOOT button) as a wake source
        esp_idf_sys::esp_sleep_enable_ext0_wakeup(
            esp_idf_sys::gpio_num_t_GPIO_NUM_0, 0); // Wake on low level
        
        // Configure timer wakeup
        esp_idf_sys::esp_sleep_enable_timer_wakeup(60 * 1000 * 1000); // 60 seconds default
        
        // You could also configure EXT1 (multiple GPIO) wakeup
        // let mask: u64 = (1 << 0) | (1 << 2); // GPIO0 and GPIO2
        // esp_idf_sys::esp_sleep_enable_ext1_wakeup(mask, 
        //     esp_idf_sys::esp_sleep_ext1_wakeup_mode_t_ESP_EXT1_WAKEUP_ANY_HIGH);
    }
    
    Ok(())
}

fn battery_voltage_to_percentage(voltage: f32) -> u8 {
    // Simple conversion for LiPo battery
    // Adjust these values based on your battery characteristics
    if voltage >= 4.2 {
        return 100;
    } else if voltage <= 3.3 {
        return 0;
    }
    
    // Linear approximation between 3.3V (0%) and 4.2V (100%)
    ((voltage - 3.3) / 0.9 * 100.0) as u8
}
```

### Lab Exercise
- Implement a power-efficient sensor node with deep sleep
- Create adaptive duty cycling based on battery level
- Measure and optimize power consumption in different modes
- Implement wake-on-event functionality for responsive operation

---

## Lecture 6.4: Project - Battery-Optimized Sensor Network

### Objectives
- Create a complete power-efficient sensor network
- Implement adaptive power management strategies
- Balance network reliability and battery life

### Topics
1. **System Architecture**
   - Node roles and responsibilities
   - Sleep scheduling strategy
   - Network synchronization approach
   - Redundancy and reliability features

2. **Adaptive Duty Cycling**
   - Environment-based adaptation
   - Battery-level-based scheduling
   - Activity-based wake periods
   - Predictive sleep scheduling

3. **Data Aggregation and Compression**
   - Reducing transmission frequency
   - Efficient data encoding
   - Local processing to reduce communications
   - Batched transmission strategies

4. **Network Health Monitoring**
   - Battery status reporting
   - Link quality assessment
   - Node lifetime estimation
   - Failure prediction and prevention

### Project Specification: Long-Life Environmental Monitoring Network

**Requirements**
1. Create a sensor network with:
   - Multiple battery-powered sensor nodes
   - One or more gateway nodes (with external power)
   - 6+ month battery life target for sensor nodes
   
2. Implement power management with:
   - Adaptive duty cycling based on battery level
   - Deep sleep with selective wake-up sources
   - Activity-driven sensing rates
   - Low-power wireless communication
   
3. Design data handling for efficiency:
   - Local data aggregation and filtering
   - Compressed data transmission
   - Event-based reporting for significant changes
   - Historical data summarization
   
4. Include network management features:
   - Remote configuration capability
   - Battery level monitoring
   - Performance and health reporting
   - Predictive maintenance alerts

**Deliverables**
1. Complete Rust firmware for sensor nodes
2. Gateway software with management interface
3. Power consumption analysis and battery life estimates
4. Field test report and performance metrics

---

# Module 7: Final Project

## Lecture 7.1: Complete IoT System Design

### Objectives
- Design a comprehensive IoT system with both WiFi and LoRa
- Create a flexible and extensible architecture
- Implement robust data management from sensor to cloud

### Topics
1. **System Architecture Overview**
   - Component interaction design
   - Communication protocols selection
   - Data flow patterns
   - Scalability considerations

2. **Hardware Component Selection**
   - Sensor types and interfaces
   - Power supply design
   - Connectivity options
   - Enclosure and deployment considerations

3. **Software Architecture**
   - Layered design approach
   - Module interfaces and abstractions
   - Error handling strategy
   - Configuration management

4. **Security Considerations**
   - Data encryption
   - Authentication mechanisms
   - Secure boot options
   - Firmware update security

### Project Outline: Multi-Protocol IoT Ecosystem

An integrated IoT system consisting of:

1. **LoRa Sensor Network**
   - Battery-powered remote sensor nodes
   - Long-range, low-bandwidth communication
   - Adaptive power management
   
2. **WiFi Gateway**
   - Bridge between LoRa and Internet
   - Local data processing and storage
   - Web dashboard for monitoring
   - OTA update distribution
   
3. **Cloud Integration**
   - Data aggregation and analysis
   - Remote configuration management
   - Alert and notification system
   - Historical data visualization

4. **Management Interface**
   - Device provisioning workflow
   - Network health monitoring
   - Firmware update mechanism
   - Configuration management tools

---

## Lecture 7.2: Data Collection, Processing, and Visualization

### Objectives
- Implement efficient data processing pipelines
- Create effective visualization interfaces
- Design suitable storage solutions for time-series data

### Topics
1. **Sensor Data Processing**
   - Calibration and normalization
   - Filtering and smoothing algorithms
   - Outlier detection and handling
   - Feature extraction techniques

2. **Edge Computing Patterns**
   - Local data processing strategies
   - Distributed computing models
   - Resource-constrained algorithms
   - Pre-processing for transmission efficiency

3. **Time-Series Data Storage**
   - Data models for IoT applications
   - Efficient storage formats
   - Retention policies
   - Compression techniques

4. **Visualization Techniques**
   - Real-time dashboards
   - Trend analysis displays
   - Alert visualization
   - Mobile-friendly interfaces

### Code Example: Dashboard Implementation
```rust
use esp_idf_hal::delay::FreeRtos;
use esp_idf_svc::http::server::{Configuration, EspHttpServer};
use esp_idf_svc::io::Write;
use esp_idf_sys::EspError;
use std::sync::{Arc, Mutex};
use std::collections::VecDeque;
use embedded_svc::http::Method;
use embedded_svc::http::server::Request;
use serde::Serialize;

// Maximum samples to store for each sensor
const MAX_HISTORY_SAMPLES: usize = 100;

// Sensor data structure
#[derive(Clone, Debug, Serialize)]
struct SensorReading {
    timestamp: u64,
    temperature: f32,
    humidity: f32,
    pressure: f32,
    battery: u8,
}

// Time series database
#[derive(Clone)]
struct TimeSeriesDB {
    readings: VecDeque<SensorReading>,
}

impl TimeSeriesDB {
    fn new() -> Self {
        Self {
            readings: VecDeque::with_capacity(MAX_HISTORY_SAMPLES),
        }
    }
    
    fn add_reading(&mut self, reading: SensorReading) {
        if self.readings.len() >= MAX_HISTORY_SAMPLES {
            self.readings.pop_front();
        }
        self.readings.push_back(reading);
    }
    
    fn get_readings(&self) -> Vec<SensorReading> {
        self.readings.iter().cloned().collect()
    }
    
    fn get_latest(&self) -> Option<SensorReading> {
        self.readings.back().cloned()
    }
    
    fn get_statistics(&self) -> SensorStatistics {
        let mut stats = SensorStatistics {
            count: self.readings.len(),
            avg_temperature: 0.0,
            avg_humidity: 0.0,
            avg_pressure: 0.0,
            min_temperature: f32::MAX,
            max_temperature: f32::MIN,
            min_humidity: f32::MAX,
            max_humidity: f32::MIN,
            min_pressure: f32::MAX,
            max_pressure: f32::MIN,
        };
        
        if self.readings.is_empty() {
            return stats;
        }
        
        let mut temp_sum = 0.0;
        let mut humidity_sum = 0.0;
        let mut pressure_sum = 0.0;
        
        for reading in &self.readings {
            temp_sum += reading.temperature;
            humidity_sum += reading.humidity;
            pressure_sum += reading.pressure;
            
            stats.min_temperature = stats.min_temperature.min(reading.temperature);
            stats.max_temperature = stats.max_temperature.max(reading.temperature);
            stats.min_humidity = stats.min_humidity.min(reading.humidity);
            stats.max_humidity = stats.max_humidity.max(reading.humidity);
            stats.min_pressure = stats.min_pressure.min(reading.pressure);
            stats.max_pressure = stats.max_pressure.max(reading.pressure);
        }
        
        let count = self.readings.len() as f32;
        stats.avg_temperature = temp_sum / count;
        stats.avg_humidity = humidity_sum / count;
        stats.avg_pressure = pressure_sum / count;
        
        stats
    }
}

// Statistics for sensor readings
#[derive(Debug, Serialize)]
struct SensorStatistics {
    count: usize,
    avg_temperature: f32,
    avg_humidity: f32,
    avg_pressure: f32,
    min_temperature: f32,
    max_temperature: f32,
    min_humidity: f32,
    max_humidity: f32,
    min_pressure: f32,
    max_pressure: f32,
}

fn main() -> Result<(), EspError> {
    // Initialize ESP-IDF
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    
    log::info!("Starting IoT dashboard server");
    
    // Create time series database
    let db = Arc::new(Mutex::new(TimeSeriesDB::new()));
    
    // Create HTTP server
    let server_config = Configuration::default();
    let mut server = EspHttpServer::new(&server_config)?;
    
    // Set up API endpoints
    setup_api_endpoints(&mut server, Arc::clone(&db))?;
    
    // Set up web dashboard
    setup_dashboard(&mut server)?;
    
    // Simulate sensor readings
    let db_clone = Arc::clone(&db);
    std::thread::spawn(move || {
        log::info!("Starting sensor simulation");
        
        let mut counter = 0;
        loop {
            // Generate simulated sensor reading
            let timestamp = esp_idf_sys::esp_timer_get_time() as u64 / 1000; // milliseconds
            let reading = SensorReading {
                timestamp,
                temperature: 22.0 + (counter % 20) as f32 / 10.0,
                humidity: 50.0 + (counter % 10) as f32,
                pressure: 1013.0 + (counter % 30) as f32 / 10.0,
                battery: 90 - (counter % 30) as u8,
            };
            
            // Add to database
            {
                let mut db = db_clone.lock().unwrap();
                db.add_reading(reading);
            }
            
            counter += 1;
            FreeRtos::delay_ms(5000); // Simulate reading every 5 seconds
        }
    });
    
    log::info!("Server started, visit http://192.168.1.1/ in your browser");
    
    // Keep main thread alive
    loop {
        FreeRtos::delay_ms(1000);
    }
}

fn setup_api_endpoints(server: &mut EspHttpServer, db: Arc<Mutex<TimeSeriesDB>>) -> Result<(), EspError> {
    // API endpoint for the latest reading
    let db_clone = Arc::clone(&db);
    server.fn_handler("/api/latest", Method::Get, move |_req| {
        let db = db_clone.lock().unwrap();
        if let Some(reading) = db.get_latest() {
            let json = serde_json::to_string(&reading).unwrap_or_else(|_| "{}".to_string());
            Ok(resp)
                .status(200)
                .header("Content-Type", "application/json")
                .body(json.as_bytes())
        } else {
            Ok(resp)
                .status(404)
                .header("Content-Type", "application/json")
                .body(b"{\"error\":\"No data available\"}")
        }
    })?;
    
    // API endpoint for all readings
    let db_clone = Arc::clone(&db);
    server.fn_handler("/api/readings", Method::Get, move |_req| {
        let db = db_clone.lock().unwrap();
        let readings = db.get_readings();
        let json = serde_json::to_string(&readings).unwrap_or_else(|_| "[]".to_string());
        
        Ok(resp)
            .status(200)
            .header("Content-Type", "application/json")
            .body(json.as_bytes())
    })?;
    
    // API endpoint for statistics
    let db_clone = Arc::clone(&db);
    server.fn_handler("/api/stats", Method::Get, move |_req| {
        let db = db_clone.lock().unwrap();
        let stats = db.get_statistics();
        let json = serde_json::to_string(&stats).unwrap_or_else(|_| "{}".to_string());
        
        Ok(resp)
            .status(200)
            .header("Content-Type", "application/json")
            .body(json.as_bytes())
    })?;
    
    Ok(())
}

fn setup_dashboard(server: &mut EspHttpServer) -> Result<(), EspError> {
    // Serve the dashboard HTML page
    server.fn_handler("/", Method::Get, |_req| {
        const DASHBOARD_HTML: &str = r#"
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 IoT Dashboard</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 0;
            background-color: #f5f5f5;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            padding: 20px;
        }
        header {
            background-color: #333;
            color: white;
            padding: 10px 20px;
            text-align: center;
        }
        .card {
            background-color: white;
            border-radius: 5px;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
            margin-bottom: 20px;
            padding: 20px;
        }
        .card h2 {
            margin-top: 0;
            border-bottom: 1px solid #eee;
            padding-bottom: 10px;
        }
        .current-readings {
            display: flex;
            flex-wrap: wrap;
            gap: 20px;
        }
        .reading-box {
            flex: 1;
            min-width: 200px;
            text-align: center;
            padding: 20px;
            border-radius: 5px;
            background-color: #f9f9f9;
        }
        .reading-value {
            font-size: 2em;
            font-weight: bold;
            margin: 10px 0;
        }
        .reading-unit {
            color: #666;
        }
        .battery-indicator {
            width: 100%;
            background-color: #ddd;
            border-radius: 10px;
            margin-top: 10px;
        }
        .battery-level {
            height: 20px;
            border-radius: 10px;
            background-color: #4CAF50;
        }
        .chart-container {
            height: 300px;
            margin-top: 20px;
        }
    </style>
</head>
<body>
    <header>
        <h1>ESP32 IoT Dashboard</h1>
    </header>
    
    <div class="container">
        <div class="card">
            <h2>Current Readings</h2>
            <div class="current-readings">
                <div class="reading-box">
                    <div>Temperature</div>
                    <div class="reading-value" id="temperature">--</div>
                    <div class="reading-unit">°C</div>
                </div>
                <div class="reading-box">
                    <div>Humidity</div>
                    <div class="reading-value" id="humidity">--</div>
                    <div class="reading-unit">%</div>
                </div>
                <div class="reading-box">
                    <div>Pressure</div>
                    <div class="reading-value" id="pressure">--</div>
                    <div class="reading-unit">hPa</div>
                </div>
                <div class="reading-box">
                    <div>Battery</div>
                    <div class="reading-value" id="battery">--</div>
                    <div class="reading-unit">%</div>
                    <div class="battery-indicator">
                        <div class="battery-level" id="battery-level" style="width: 0%"></div>
                    </div>
                </div>
            </div>
        </div>
        
        <div class="card">
            <h2>Temperature History</h2>
            <div class="chart-container">
                <canvas id="temp-chart"></canvas>
            </div>
        </div>
        
        <div class="card">
            <h2>Humidity & Pressure History</h2>
            <div class="chart-container">
                <canvas id="hum-press-chart"></canvas>
            </div>
        </div>
        
        <div class="card">
            <h2>Statistics</h2>
            <table id="stats-table" style="width: 100%; border-collapse: collapse;">
                <tr>
                    <th style="text-align: left; padding: 8px; border-bottom: 1px solid #ddd;">Metric</th>
                    <th style="text-align: right; padding: 8px; border-bottom: 1px solid #ddd;">Average</th>
                    <th style="text-align: right; padding: 8px; border-bottom: 1px solid #ddd;">Minimum</th>
                    <th style="text-align: right; padding: 8px; border-bottom: 1px solid #ddd;">Maximum</th>
                </tr>
                <tr>
                    <td style="padding: 8px; border-bottom: 1px solid #ddd;">Temperature (°C)</td>
                    <td style="text-align: right; padding: 8px; border-bottom: 1px solid #ddd;" id="avg-temp">--</td>
                    <td style="text-align: right; padding: 8px; border-bottom: 1px solid #ddd;" id="min-temp">--</td>
                    <td style="text-align: right; padding: 8px; border-bottom: 1px solid #ddd;" id="max-temp">--</td>
                </tr>
                <tr>
                    <td style="padding: 8px; border-bottom: 1px solid #ddd;">Humidity (%)</td>
                    <td style="text-align: right; padding: 8px; border-bottom: 1px solid #ddd;" id="avg-hum">--</td>
                    <td style="text-align: right; padding: 8px; border-bottom: 1px solid #ddd;" id="min-hum">--</td>
                    <td style="text-align: right; padding: 8px; border-bottom: 1px solid #ddd;" id="max-hum">--</td>
                </tr>
                <tr>
                    <td style="padding: 8px;">Pressure (hPa)</td>
                    <td style="text-align: right; padding: 8px;" id="avg-press">--</td>
                    <td style="text-align: right; padding: 8px;" id="min-press">--</td>
                    <td style="text-align: right; padding: 8px;" id="max-press">--</td>
                </tr>
            </table>
        </div>
    </div>

    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <script>
        // Initialize charts
        const tempCtx = document.getElementById('temp-chart').getContext('2d');
        const tempChart = new Chart(tempCtx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [{
                    label: 'Temperature (°C)',
                    data: [],
                    borderColor: 'rgb(255, 99, 132)',
                    backgroundColor: 'rgba(255, 99, 132, 0.1)',
                    tension: 0.1,
                    fill: true
                }]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                scales: {
                    y: {
                        beginAtZero: false
                    }
                }
            }
        });

        const humPressCtx = document.getElementById('hum-press-chart').getContext('2d');
        const humPressChart = new Chart(humPressCtx, {
            type: 'line',
            data: {
                labels: [],
                datasets: [
                    {
                        label: 'Humidity (%)',
                        data: [],
                        borderColor: 'rgb(54, 162, 235)',
                        backgroundColor: 'rgba(54, 162, 235, 0.1)',
                        tension: 0.1,
                        yAxisID: 'y'
                    },
                    {
                        label: 'Pressure (hPa)',
                        data: [],
                        borderColor: 'rgb(75, 192, 192)',
                        backgroundColor: 'rgba(75, 192, 192, 0.1)',
                        tension: 0.1,
                        yAxisID: 'y1'
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                scales: {
                    y: {
                        type: 'linear',
                        display: true,
                        position: 'left',
                        beginAtZero: false
                    },
                    y1: {
                        type: 'linear',
                        display: true,
                        position: 'right',
                        beginAtZero: false,
                        grid: {
                            drawOnChartArea: false
                        }
                    }
                }
            }
        });

        // Function to update dashboard
        function updateDashboard() {
            // Fetch latest reading
            fetch('/api/latest')
                .then(response => response.json())
                .then(data => {
                    // Update current readings
                    document.getElementById('temperature').textContent = data.temperature.toFixed(1);
                    document.getElementById('humidity').textContent = data.humidity.toFixed(0);
                    document.getElementById('pressure').textContent = data.pressure.toFixed(1);
                    document.getElementById('battery').textContent = data.battery;
                    
                    // Update battery indicator
                    document.getElementById('battery-level').style.width = data.battery + '%';
                    
                    // Change battery color based on level
                    const batteryLevel = document.getElementById('battery-level');
                    if (data.battery < 20) {
                        batteryLevel.style.backgroundColor = '#f44336'; // Red
                    } else if (data.battery < 50) {
                        batteryLevel.style.backgroundColor = '#ff9800'; // Orange
                    } else {
                        batteryLevel.style.backgroundColor = '#4CAF50'; // Green
                    }
                })
                .catch(error => console.error('Error fetching latest reading:', error));
            
            // Fetch all readings for chart
            fetch('/api/readings')
                .then(response => response.json())
                .then(data => {
                    // Format data for charts
                    const timestamps = data.map(reading => {
                        const date = new Date(reading.timestamp);
                        return date.toLocaleTimeString();
                    });
                    const temperatures = data.map(reading => reading.temperature);
                    const humidities = data.map(reading => reading.humidity);
                    const pressures = data.map(reading => reading.pressure);
                    
                    // Update temperature chart
                    tempChart.data.labels = timestamps;
                    tempChart.data.datasets[0].data = temperatures;
                    tempChart.update();
                    
                    // Update humidity & pressure chart
                    humPressChart.data.labels = timestamps;
                    humPressChart.data.datasets[0].data = humidities;
                    humPressChart.data.datasets[1].data = pressures;
                    humPressChart.update();
                })
                .catch(error => console.error('Error fetching readings:', error));
            
            // Fetch statistics
            fetch('/api/stats')
                .then(response => response.json())
                .then(data => {
                    // Update statistics table
                    document.getElementById('avg-temp').textContent = data.avg_temperature.toFixed(1);
                    document.getElementById('min-temp').textContent = data.min_temperature.toFixed(1);
                    document.getElementById('max-temp').textContent = data.max_temperature.toFixed(1);
                    
                    document.getElementById('avg-hum').textContent = data.avg_humidity.toFixed(0);
                    document.getElementById('min-hum').textContent = data.min_humidity.toFixed(0);
                    document.getElementById('max-hum').textContent = data.max_humidity.toFixed(0);
                    
                    document.getElementById('avg-press').textContent = data.avg_pressure.toFixed(1);
                    document.getElementById('min-press').textContent = data.min_pressure.toFixed(1);
                    document.getElementById('max-press').textContent = data.max_pressure.toFixed(1);
                })
                .catch(error => console.error('Error fetching statistics:', error));
        }
        
        // Initial update
        updateDashboard();
        
        // Set interval for updates
        setInterval(updateDashboard, 5000);
    </script>
</body>
</html>
        "#;
        
        Ok(resp)
            .status(200)
            .header("Content-Type", "text/html")
            .body(DASHBOARD_HTML.as_bytes())
    })?;
    
    Ok(())
}
```

### Lab Exercise
- Implement a sensor data processing pipeline
- Create a web-based visualization dashboard
- Design an efficient time-series data storage solution
- Implement data analytics for trend detection

---

## Lecture 7.3: Implementing OTA Updates

### Objectives
- Design and implement secure over-the-air updates
- Create reliable firmware upgrade procedures
- Handle update failures and rollbacks

### Topics
1. **OTA System Architecture**
   - Update server design
   - Firmware versioning strategy
   - Update package structure
   - Distribution mechanisms

2. **Secure Update Process**
   - Package signing and verification
   - Secure transmission
   - Privilege management
   - Anti-rollback protection

3. **Reliable Update Procedures**
   - A/B partitioning
   - Staged updates
   - Validation mechanisms
   - Rollback strategies

4. **Implementation Techniques**
   - Update agent design
   - Flash management
   - Storage requirements
   - Resource constraints

### Code Example: OTA Update Implementation
```rust
use anyhow::{anyhow, Result};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::http::client::{Configuration as HttpConfiguration, EspHttpConnection};
use esp_idf_svc::nvs::{EspNvs, EspDefaultNvsPartition, NvsOpen};
use esp_idf_svc::ota::{EspOta};
use esp_idf_svc::wifi::{BlockingWifi, ClientConfiguration, Configuration, EspWifi};
use embedded_svc::http::client::Client as HttpClient;
use embedded_svc::io::Read;
use std::time::Duration;

// OTA configuration
const FIRMWARE_VERSION: &str = "1.0.0";
const OTA_SERVER_URL: &str = "https://your-server.com/firmware";
const OTA_CHECK_INTERVAL_SEC: u64 = 3600; // Check every hour

// Firmware update progress callback
fn update_progress(progress: u32, total: u32) {
    let percentage = (progress * 100) / total;
    log::info!("Update progress: {}% ({}/{})", percentage, progress, total);
}

fn main() -> Result<()> {
    // Initialize ESP-IDF
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    
    log::info!("Starting OTA update example");
    log::info!("Current firmware version: {}", FIRMWARE_VERSION);
    
    // Get peripherals and event loop
    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    
    // Initialize NVS storage for OTA
    let mut storage = EspNvs::new(nvs.clone(), "ota", NvsOpen::ReadWrite)?;
    
    // Read last update attempt status
    match storage.get_u32("last_result") {
        Ok(Some(0)) => log::info!("Last update was successful"),
        Ok(Some(code)) => log::warn!("Last update failed with code: {}", code),
        _ => log::info!("No previous update recorded"),
    }
    
    // Connect to WiFi
    log::info!("Connecting to WiFi");
    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs))?,
        sys_loop,
    )?;
    
    let wifi_configuration = Configuration::Client(ClientConfiguration {
        ssid: "YourWiFiSSID".into(),
        password: "YourWiFiPassword".into(),
        ..Default::default()
    });
    
    wifi.set_configuration(&wifi_configuration)?;
    wifi.start()?;
    wifi.connect()?;
    wifi.wait_netif_up()?;
    
    log::info!("WiFi connected!");
    
    // Create HTTP client for update checks
    let http_config = HttpConfiguration {
        timeout: Some(Duration::from_secs(30)),
        ..Default::default()
    };
    
    let client = HttpClient::wrap(EspHttpConnection::new(&http_config)?);
    
    // Create OTA object
    let mut ota = EspOta::new()?;
    
    // Main loop - periodically check for updates
    let mut last_update_check = 0u64;
    
    loop {
        // Main application functionality
        // ...
        
        // Check for updates periodically
        let now = esp_idf_sys::esp_timer_get_time() as u64 / 1_000_000;  // seconds
        if now - last_update_check >= OTA_CHECK_INTERVAL_SEC {
            log::info!("Checking for firmware updates...");
            
            match check_for_update(&client, FIRMWARE_VERSION) {
                Ok(Some(update_info)) => {
                    log::info!("New firmware available: v{}", update_info.version);
                    log::info!("Downloading and installing update...");
                    
                    // Perform the update
                    match perform_update(&mut ota, &client, &update_info, update_progress) {
                        Ok(_) => {
                            log::info!("Update downloaded successfully, rebooting...");
                            
                            // Record successful download
                            let _ = storage.set_u32("last_result", 0);
                            let _ = storage.commit();
                            
                            // Wait a moment before rebooting
                            std::thread::sleep(Duration::from_secs(3));
                            
                            // Reboot to apply the update
                            esp_idf_sys::esp_restart();
                        }
                        Err(e) => {
                            log::error!("Update failed: {}", e);
                            
                            // Record failure
                            let _ = storage.set_u32("last_result", 1);
                            let _ = storage.commit();
                        }
                    }
                }
                Ok(None) => {
                    log::info!("No updates available");
                }
                Err(e) => {
                    log::error!("Error checking for updates: {}", e);
                }
            }
            
            last_update_check = now;
        }
        
        // Sleep to prevent busy looping
        std::thread::sleep(Duration::from_secs(1));
    }
}

// Update information
struct UpdateInfo {
    version: String,
    url: String,
    size: usize,
    checksum: String,
}

// Check if an update is available
fn check_for_update(client: &impl embedded_svc::http::client::Client, current_version: &str) -> Result<Option<UpdateInfo>> {
    let url = format!("{}/check?version={}", OTA_SERVER_URL, current_version);
    
    let mut response = client.get(&url)?.submit()?;
    let status = response.status();
    
    if status != 200 {
        return Err(anyhow!("Server returned status {}", status));
    }
    
    // Read the response
    let mut buffer = [0u8; 1024];
    let mut cursor = 0;
    
    while let Ok(size) = response.read(&mut buffer[cursor..]) {
        if size == 0 {
            break;
        }
        cursor += size;
        if cursor >= buffer.len() {
            break;
        }
    }
    
    // Parse the JSON response
    let json_str = std::str::from_utf8(&buffer[0..cursor])?;
    
    // Simple JSON parsing (in a real application, use a JSON library)
    if json_str.contains("\"update_available\":true") {
        // Parse version, URL, size, and checksum from JSON
        // This is a simplified example - in a real application use proper JSON parsing
        
        let version = extract_json_string(json_str, "version")
            .ok_or_else(|| anyhow!("Missing version in response"))?;
        let url = extract_json_string(json_str, "url")
            .ok_or_else(|| anyhow!("Missing URL in response"))?;
        let size = extract_json_number(json_str, "size")
            .ok_or_else(|| anyhow!("Missing size in response"))?;
        let checksum = extract_json_string(json_str, "checksum")
            .ok_or_else(|| anyhow!("Missing checksum in response"))?;
        
        Ok(Some(UpdateInfo {
            version,
            url,
            size: size as usize,
            checksum,
        }))
    } else {
        Ok(None)
    }
}

// Very simple JSON string extraction - in a real application use a proper JSON parser
fn extract_json_string(json: &str, key: &str) -> Option<String> {
    let pattern = format!("\"{}\":\"", key);
    let start = json.find(&pattern)? + pattern.len();
    let end = json[start..].find('"')? + start;
    Some(json[start..end].to_string())
}

fn extract_json_number(json: &str, key: &str) -> Option<u32> {
    let pattern = format!("\"{}\":", key);
    let start = json.find(&pattern)? + pattern.len();
    let end = json[start..].find(|c: char| !c.is_ascii_digit())? + start;
    json[start..end].parse().ok()
}

// Perform the firmware update
fn perform_update(
    ota: &mut EspOta,
    client: &impl embedded_svc::http::client::Client,
    update_info: &UpdateInfo,
    progress_callback: fn(u32, u32)
) -> Result<()> {
    // Begin OTA update
    log::info!("Starting OTA update process");
    ota.begin(update_info.size as u32)?;
    
    // Download and write the firmware
    let mut response = client.get(&update_info.url)?.submit()?;
    let status = response.status();
    
    if status != 200 {
        return Err(anyhow!("Server returned status {} for firmware download", status));
    }
    
    // Buffer for downloading firmware
    let mut buffer = [0u8; 4096];
    let mut bytes_written = 0u32;
    
    // Read and write firmware in chunks
    loop {
        match response.read(&mut buffer) {
            Ok(0) => break, // End of data
            Ok(size) => {
                // Write the data to the OTA partition
                ota.write(&buffer[0..size])?;
                
                bytes_written += size as u32;
                progress_callback(bytes_written, update_info.size as u32);
            }
            Err(e) => {
                return Err(anyhow!("Error reading firmware data: {}", e));
            }
        }
    }
    
    // Verify and finish OTA update
    log::info!("Verifying firmware...");
    if ota.end()? {
        log::info!("Firmware verified successfully");
        Ok(())
    } else {
        Err(anyhow!("Firmware verification failed"))
    }
}
```

### Lab Exercise
- Implement an OTA update system for ESP32
- Create a secure firmware distribution service
- Test update reliability and rollback procedures
- Measure update performance and impact on system operation

---

## Lecture 7.4: System Hardening and Security

### Objectives
- Implement robust security practices for IoT systems
- Protect against common vulnerabilities
- Ensure secure communication and data handling

### Topics
1. **Threat Modeling**
   - Identifying potential attack vectors
   - Risk assessment methodologies
   - Security requirements definition
   - Defensive design approach

2. **Secure Communication**
   - TLS/SSL implementation
   - Certificate management
   - Key storage and handling
   - Secure protocols selection

3. **Device Security**
   - Secure boot implementation
   - Flash encryption
   - Debug port protection
   - Anti-tampering measures

4. **Data Protection**
   - Sensitive data handling
   - Encryption at rest
   - Secure storage implementation
   - Privacy considerations

### Code Example: Secure Communication Implementation
```rust
use anyhow::Result;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{BlockingWifi, ClientConfiguration, Configuration, EspWifi};
use esp_idf_svc::http::client::{Configuration as HttpConfiguration, EspHttpConnection};
use esp_idf_svc::tls::{X509, EspTlsStack, TlsClientConfig};
use embedded_svc::http::Method;
use embedded_svc::http::client::Client as HttpClient;
use embedded_svc::io::Read;
use std::time::Duration;

// Root certificate for server verification
const ROOT_CERT: &str = r#"-----BEGIN CERTIFICATE-----
MIIDSjCCAjKgAwIBAgIQRK+wgNajJ7qJMDmGLvhAazANBgkqhkiG9w0BAQUFADA/
MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT
DkRTVCBSb290IENBIFgzMB4XDTAwMDkzMDIxMTIxOVoXDTIxMDkzMDE0MDExNVow
...
more certificate data here
...
+------END CERTIFICATE-----
"#;

// API keys (in a real application, store these securely)
const API_KEY: &str = "your-api-key";
const API_SECRET: &str = "your-api-secret";

fn main() -> Result<()> {
    // Initialize ESP-IDF
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    
    log::info!("Starting secure communication example");
    
    // Get peripherals and event loop
    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    
    // Connect to WiFi
    log::info!("Connecting to WiFi");
    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(peripherals.modem, sys_loop.clone(), Some(nvs))?,
        sys_loop,
    )?;
    
    let wifi_configuration = Configuration::Client(ClientConfiguration {
        ssid: "YourWiFiSSID".into(),
        password: "YourWiFiPassword".into(),
        ..Default::default()
    });
    
    wifi.set_configuration(&wifi_configuration)?;
    wifi.start()?;
    wifi.connect()?;
    wifi.wait_netif_up()?;
    
    log::info!("WiFi connected!");
    
    // Create TLS configuration with root certificate
    let root_cert = X509::pem_until_nul(ROOT_CERT.as_bytes())?;
    let tls_config = TlsClientConfig::new()
        .alpn_protocols(&["http/1.1"])
        .add_root_certificate(root_cert);
    
    // Create HTTP client with TLS
    let http_config = HttpConfiguration {
        timeout: Some(Duration::from_secs(10)),
        use_global_ca_store: false,
        crt_bundle_attach: Some(esp_idf_sys::esp_crt_bundle_attach),
        client_certificate: None,
        client_private_key: None,
        server_certificate: Some(X509::pem_until_nul(ROOT_CERT.as_bytes())?),
        ..Default::default()
    };
    
    let tls = EspTlsStack::new_with_crt_bundle()?;
    let client = HttpClient::wrap(EspHttpConnection::new_with_tls(&http_config, tls)?);
    
    // Make secure API request
    log::info!("Making secure API request");
    
    // Current timestamp for request signing
    let timestamp = esp_idf_sys::esp_timer_get_time() / 1000000;
    
    // Create signature (in a real application, use proper HMAC)
    let signature = format!("{}:{}", API_KEY, timestamp);
    
    // Secure API endpoint
    let url = "https://api.example.com/data";
    
    // Create and send request
    let mut request = client.request(Method::Get, url)?;
    
    // Add authorization headers
    request.header("X-API-Key", API_KEY)?;
    request.header("X-Timestamp", &timestamp.to_string())?;
    request.header("X-Signature", &signature)?;
    
    // Send request and get response
    let mut response = request.submit()?;
    let status = response.status();
    
    log::info!("Response status: {}", status);
    
    // Read and process response
    if status == 200 {
        let mut buffer = [0u8; 1024];
        let mut cursor = 0;
        
        while let Ok(size) = response.read(&mut buffer[cursor..]) {
            if size == 0 {
                break;
            }
            cursor += size;
            if cursor >= buffer.len() {
                break;
            }
        }
        
        let response_data = std::str::from_utf8(&buffer[0..cursor])?;
        log::info!("Received data: {}", response_data);
        
        // Process the data
        // ...
    } else {
        log::warn!("API request failed with status: {}", status);
    }
    
    // Secure data storage example
    store_sensitive_data("username", "secure_user")?;
    store_sensitive_data("password", "secure_password")?;
    
    // Retrieve secure data
    if let Some(username) = get_sensitive_data("username")? {
        log::info!("Retrieved username: {}", username);
    }
    
    Ok(())
}

// Function to securely store sensitive data
fn store_sensitive_data(key: &str, value: &str) -> Result<()> {
    // In a real application, encrypt this data before storing
    // Here we just use NVS, but you should implement proper encryption
    
    let nvs = EspDefaultNvsPartition::take()?;
    let mut storage = esp_idf_svc::nvs::EspNvs::new(nvs, "secure_storage", esp_idf_svc::nvs::NvsOpen::ReadWrite)?;
    
    log::info!("Storing sensitive data for key: {}", key);
    storage.set_str(key, value)?;
    
    Ok(())
}

// Function to retrieve sensitive data
fn get_sensitive_data(key: &str) -> Result<Option<String>> {
    // In a real application, decrypt this data after retrieving
    
    let nvs = EspDefaultNvsPartition::take()?;
    let storage = esp_idf_svc::nvs::EspNvs::new(nvs, "secure_storage", esp_idf_svc::nvs::NvsOpen::ReadOnly)?;
    
    log::info!("Retrieving sensitive data for key: {}", key);
    let value = storage.get_str(key)?;
    
    Ok(value)
}
```

### Lab Exercise
- Implement secure communication with TLS
- Create a secure data storage mechanism
- Perform a security assessment of your IoT system
- Implement secure boot and flash encryption

---

# Assessment Strategy

- **Weekly Coding Assignments (40%)**
  - Short practical exercises focusing on specific concepts
  - Code quality and documentation assessment
  - Unit testing and error handling evaluation
  - Peer code review participation

- **Mid-Course Project (20%)**
  - Implementation of a complete subsystem
  - Technical documentation and architecture description
  - Performance and power consumption analysis
  - Oral presentation and demonstration

- **Final Project Implementation (30%)**
  - End-to-end IoT system development
  - Integration of WiFi and LoRa technologies
  - Documentation and testing strategy
  - Live demonstration and code walkthrough

- **Documentation and Code Quality (10%)**
  - Code style and organization
  - Technical documentation quality
  - API design and abstraction quality
  - Project organization and maintainability

---

# Resources and References

## Books
- "Rust Programming Language" (The Rust Book) - Official Rust Documentation
- "Programming Rust: Fast, Safe Systems Development" - Jim Blandy, Jason Orendorff
- "Embedded Rust" - Jorge Aparicio
- "IoT Development with Rust" - Joseph Yiu

## Online Resources
- [Rust Official Documentation](https://doc.rust-lang.org/)
- [Embedded Rust Book](https://docs.rust-embedded.org/book/)
- [ESP-RS Repository](https://github.com/esp-rs)
- [ESP32 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)

## Development Tools
- ESP-IDF (ESP32 IoT Development Framework)
- Rust Compiler and Cargo Package Manager
- VS Code with Rust-Analyzer Extension
- ESP-RS Tools for Rust on ESP32

## Hardware Documentation
- ESP32 D0WDQ6 Datasheet
- WiFi-LoRa-32-V2 Board Schematics
- SX1276/SX1278 LoRa Transceiver Datasheet
- Sensor Datasheets (as needed for projects)

---

This course is designed to provide a comprehensive introduction to Rust programming on the ESP32 platform, with an emphasis on practical skills and real-world applications. Students will gain hands-on experience with memory-safe embedded development, wireless communication technologies, and IoT system design principles.
    