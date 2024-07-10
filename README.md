# Integrated ADAS for Enhanced Road Safety

**Project Overview:**
This project aims to enhance road safety through the development of an Integrated Advanced Driver Assistance System (ADAS) leveraging Vehicle-to-Vehicle (V2V) communication. The system facilitates real-time data exchange between vehicles, enabling advanced collision avoidance and providing a safer driving experience.

## Key Features
- **V2V Communication:**
  - Real-time exchange of speed, position, and direction data between vehicles.
  - Enhances situational awareness and enables proactive collision avoidance.

- **Collision Avoidance Module:**
  - Analyzes incoming data to predict potential collisions.
  - Provides timely alerts to drivers and assists in taking corrective actions.

- **Sensor Integration:**
  - Integrates LIDAR for comprehensive obstacle and vehicle detection.
  - Ensures accurate and reliable data for system operations.

- **Simulation and Testing:**
  - Extensive simulations conducted to test system performance, reliability, and scalability.
  - Demonstrated significant improvements in driving safety in various scenarios.

- **Scalability and Interoperability:**
  - Designed to be scalable and compatible with different vehicle models and communication protocols.
  - Ensures broad applicability and ease of integration.

- **User-Friendly Interface:**
  - Developed an intuitive interface for easy system updates and interactions.
  - Provides a seamless user experience for drivers.

## Technologies Used
- **Hardware:** STM32F411 (Black Pill), nRF24L01 module, RPLIDAR, Raspberry Pi Camera (OV5647), Servo 995G, DC Motor (JGA25-370), Motor Driver L298N Module, DC-DC Buck Converter (LM2596).
- **Software:** FreeRTOS, GIT, GitHub.
  
## How to Use

### System Setup
1. Assemble the hardware components as per the provided schematics.
2. Connect the STM32F411, nRF24L01 module and other peripherals.

### Software Installation
1. Clone the GitHub repository to your local machine:
    ```bash
    git clone https://github.com/Aymann00/Integrated-ADAS.git
    ```
2. Navigate to the project directory:
    ```bash
    cd Integrated-ADAS
    ```

### Dependencies
1. Install the necessary software tools and libraries for STM32 development (e.g., STM32CubeIDE, STM32CubeMX).
2. Import the project into your development environment.

### Running the System
1. Build and flash the firmware to the STM32F411.
2. The system will initialize and start the V2V communication and collision avoidance modules.

### Testing and Simulation
1. Use the provided simulation scripts to test the system's performance under different scenarios.
2. Adjust the parameters as needed to optimize performance.

## Contributing
Contributions are welcome! Please fork the repository and submit pull requests for review. Report any issues or feature requests in the GitHub issues section.

## License
This project is licensed under the MIT License. See the LICENSE file for more details.

## Contact
For any queries or support, please contact the project supervisors via their respective email addresses.

By following these steps, you can set up, configure, and utilize the Integrated ADAS system for enhanced road safety. The project demonstrates the feasibility of using advanced communication and sensor technologies to improve driving safety and reduce accidents.
