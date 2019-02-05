![img](/pics/header.jpg)



### SYSC 4805 - Computer Systems Design Lab



# **Project Proposal**



## Group #9



## Students:

Theo Hronowsky	101008637

Emmanuel Oluyomi	100953361

Zein Hajj-Ali	101020677

Hussain Aljabri	100935515

Ahmad Ayyoub	100954214



## Objectives

The goal of the computer systems design lab as part of the SYSC 4805 project is to design and implement a robotic platform. The system will contain various sensors and the associated software that processes sensor data and drives the robot to solve a task. The task that our robot will accomplish is self-balancing. The robot will use its motors to keep itself upright, without any other aid. If this is achieved, we will also be adding a simple line-following capability. This will be implemented using a real robot platform rather than using the simulation software, Vrep. Reasons as to why the group decided to build a physical robot rather than use the simulation software was that using a physical model of the robot would translate to real world application as well as produce a hands-on and interactive end product.



## Main Details

We decided to build a robot that balances on it&#39;s two wheels using sensor input data, and the movement of the motors to correct itself and keep upright. Our lesser part of our project is the line following aspect which should be much simpler, and is of less importance than the self-balancing project.

Since our project depends on the building of a self balancing robot, we require the use of a gyroscope. We chose to use a gyroscope rather than an accelerometer due to the fact that a gyroscope can detect rotation around a given axis. This will be useful for when we come to the stage of the project where we want to implement the line following aspect. We will also need to build a new base for our robot that is vertical and distributes the weight of all the components evenly along the middle of the robot. This way it will lessen the amount of work needed to be done by the arduino to calculate the values for the self balancing movements.

Our project uses new sensors that were not included in the system such as a gyroscope and/or an accelerometer, as well as a whole new platform for the robot. The extra sensors we need will be sourced from Graham Eatherley and the platform will be designed custom for our project and will be 3D printed using the printers available on campus.

We will be developing an algorithm to calculate the speed values needed for the motors to keep the robot upright. There are already some algorithms and programs found online that do the same job, but we will be using ideas from them to implement our own version specific to our project. We will also be developing a simple line following algorithm that uses the movement techniques from the self-balancing part of the project.

We will be evaluating the success of our project by running tests on the practical performance. We will be testing the capability of keeping upright under four main conditions:

1. Standing still or in place,

2. Being pushed down lightly,

3. While moving in a straight line, and

4. While moving on a curved path or taking turns.

These four tests will show the full capabilities of our self-balancing system. If completed successfully, we will also be testing the basic line-following implementation.

The roles and strengths of each team member and the timeline for each stage of the project are discussed below in their respective sections.

## Team Members &amp; Roles

Group #9 consists of five 4th year computer systems engineering students. The group members are: Theo Hronowsky, Zein Hajj-Ali, Emmanuel Oluyomi, Hussain Aljabri and Ahmad Ayyoub. Each member will assume the role of the project manager and will have their own associated responsibilities.  The roles of each team member is split in a way that each member of the team will participate in all aspects of project planning, management and the design and development of the end solution.

As 4th year computer system engineers, all group members have similar knowledge and experience. Experience such as embedded system design and various programming principles are a shared skill set between the team. Knowledge in microcontrollers and sensors have been acquired in previous projects and will be applied by all members. More so, all members of the group have extensive knowledge of various programming principles. That being said, the group will work as a team on all phases of the engineering design process, and as problems arise, will branch off, based on interests, to propose solutions.

### Proposed Tasks:

- **Zein Hajj-Ali:** Programming, Building Hardware, Developing testing scenarios
- **Theo Hronowsky:** Sensor programming, assembling the robot/hardware, CAD design/3-D printing
- **Emmanuel Oluyomi:** Coding of Arduino, 3-D printing, testing of program.
- **Hussain Aljabri:** Hardware connections and Arduino programming.
- **Ahmad Ayyoub:** Writing or testing the software needed to balance the robot or make it follow the line



## Project Planning steps

The computer systems design lab project will be planned in a way that it is iterative. It will be planned using a variation of the engineering design process. The robot will be created through defining, planning, design, implementing and testing phases. The team will start working on the first iteration of the project after acquiring and checking all the necessary sensors and parts.

First, the team members will collaborate to determine the needed hardware sensors that will be connected to the robot to accomplish the self-balancing and line following functionalities of the robot. For now, we know that we will need the gyroscope and accelerometer sensors to implement the functionalities. The second step is to connect these sensors with other sensors of the robot to make it move on the line. The next step is to test the sensors if they are working after we connect them with the robot.Then we proceed to writing or downloading the necessary software application code on arduino that will cause the sensors to work accordingly. Then we will need to check for any mistakes or variations from the robot. For example, we will test whether the robot is actually moving on the line while balancing itself and not falling off. In other words, we will do the integration testing of the software and hardware of the robot.We then combine the sensors and implement the required software code that will make it move in a line and balance itself at the same time. We&#39;ve organized a tentative schedule for the group to achieve the project objectives for structure.

The tentative schedule for the group is as follows:

| **Component** | **Date** | **Details** |
| --- | --- | --- |
| **Requirements Elicitation** |
| Project topic choice &amp; planning | Jan 25th - Feb 1st, 2019 | Choose topic and begin planning |
| Project proposal | Feb 1st 2019 | Brief description of the self balancing robot and platform chosen |
| **Planning/Design** |
| Project planning/design options | Feb 1st - Feb 15th, 2019 | Consider the weight of the robot and figuring out a design to maintain balanced and begin planning. |
| Acquire necessary hardware | Feb 1st - Feb 15th, 2019 | We need Gyroscope and/or accelerometer. |
| **Development/Implementation** |
| Project implementation | Feb 15th - Mar 1st, 2019 | Begin the implementation phase: Adding the sensors and composing the robot. |
| Project progress report | Mar. 3, 2019 | Description of the current progress |
| **Testing &amp; Refinement** |
| Testing and refinement | Mar. 3rd - Mar. 21, 2019 | Test the first iteration of the completed robot, implement changes that improve performance. |
| Final project presentation | scheduled between Mar. 22 and Apr. 5, 2019 | Presentation that will cover chosen components, implementation, and other project specifics. |
| Final project demonstration | Apr. 5, 2019 | Presentation of the implementation of the physical robot. |
| Peer review of individual contributions | Mar. 29, 2019 | Each student will demonstrate their personal contributions throughout the semester. |
| Robot kit return | Apr. 9, 2019 | Robot kit is returned. |
| Final Report | Apr. 9, 2019 | An updated version of the progress report, and contains all the technical details of the project. |

## Risk Evaluation

In any project, it can be expected that problems will arise. In order to minimize the impact of these setbacks, some considerations have been placed. This involves events that are uncertain to the group, and may have positive or negative effects on the project.

These risks can be identified as:

* The event that a piece of hardware is broken or does not work to specification. **Medium Risk, Medium Impact.** Solution: Source a replacement part; Remove need for it using an alternative method.
* If hardware is not sourced locally, the chance that shipping time may not be within the timeline of this project. **Low Risk, Low Impact.** Solution: Order parts locally; Order parts early in the process.
* There might be latency delay from the sensors. **Medium Risk, Low Impact.** Solution: Account for delay in code; Don&#39;t use speed dependant processes.
* Readings from the sensors aren&#39;t accurate and precise. **Medium Risk, Medium Impact.** Solution: Order replacement sensors; Account for precision in code.
* Scope of the project is ill defined and needs to be redefined later on. **Low Risk, Medium Impact.** Solution: Redefine scope of project.

Overall, the risk associated with the combination of the events is minimal in the sense that if these events occur, there are preventative measures or alternatives that can be considered.

## Project Cost and Duration

The basic equipment that we will use for the robot&#39;s hardware configuration are borrowed from Carleton University, and other sensors hardware equipment might be purchased or borrowed from Graham Eatherley if needed and available. If these sensors are not available locally, we will need to purchase these components, which will come with a cost. However, often times these components are cheap and will not have a huge impact in terms of cost.  As stated in the tentative schedule, the basic equipments will be returned at the end of the semester. It is assumed that the duration of the project will last the whole semester with a functional product developed at the end of the term.

## Hardware or sensor main components

These are the main sensors used in the robot and components:

* Gyroscope [^1]
![img](/pics/gyro.jpg)

* Accelerometer [^2]
![img](/pics/accel.jpg)

* Motor module [^3]
![img](/pics/mmodule.jpg)

This is the schematic sketch of what the robot will do when it starts to move to balance itself:

![img](/pics/schematic.jpg)

Figure 1: Self-balancing problem [^4]

**References**

[^1] _Amazon.co.uk_, 2019. [Online]. Available: https://www.amazon.co.uk/MMA7361-Accelerometer-Sensor-Acceleration-Inclination/dp/B008P6AM7W. [Accessed: 01- Feb- 2019].

[^2] &quot;Gyro Sensor&quot;, _indiamart.com_, 2019. [Online]. Available: https://www.indiamart.com/proddetail/gyro-sensor-12728771173.html. [Accessed: 01- Feb- 2019].

[^3] L. Contr, &quot;L9110S H-bridge Dual DC Stepper Motor Driver Contr | Arduino Robots &amp; electronics component in kabul afghanistan&quot;, _Nooraziz.com_, 2019. [Online]. Available: http://www.nooraziz.com/product/l9110s-h-bridge-dual-dc-stepper-motor-driver-contr/. [Accessed: 01- Feb- 2019].

[^4] &quot;S15: Self-Balancing Robot - Embedded Systems Learning Academy&quot;, _Socialledge.com_, 2019. [Online]. Available: http://socialledge.com/sjsu/index.php/S15:\_Self-Balancing\_Robot. [Accessed: 01- Feb- 2019].