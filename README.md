# microbot_wifi_driver
About

## How to setup
### Prerequisites
Before we move on ou need first of all to setup your GCS with:
- [ ] Docker
- [ ] microBot container
- [ ] VSCode
- [ ] Platformio Extension on VSCode

To do so, click [here](https://nikolaosmitropoulos.atlassian.net/wiki/pages/resumedraft.action?draftId=10354689) and follow the steps.

### Download the repo
Open a terminal, navigate to your preferable directory using `cd` and download the repo.

In my case I'm going to create a folder named `microBot` inside `Projects` and clone the repo there, so:
```
mkdir -p ~/Projects/microBot
cd ~/Projects/microBot
git clone git clone git@github.com:nikosmitrop1995/microbot_wifi_driver.git
```
### Open VScode and finish setup

What we have to to is finish setting up the GCS.

Open a terminal and type:
```
code
```

This will open VSCode. 

Then press `F1` and type `PlatformIO: PlatformIO Home` 
and press `Enter`. 

At the right side of the PlatformIO Home screen, click the button that says `Open Project`.

Navigate to the folder where you saved `microbot_wifi_driver` and click `Open`.

After a few minutes the dependencies will be installed.

### Upload the code
Before building and uploading the code we need to edit `src/main.cpp`. Open it using the `Explorer` bar.

Go to line 19 and `SSID_NAME` with the network that you want the microcontroller to be connected.

On line 20 replace `PASSWORD` with the password of the network.

Now, we can build and upload the code to the controller.

First of all connect the microcontroller with the computer that is running PlatformIO and has the `main.cpp`.

Later, press `F1`,type `PlatformIO: Build` and hit `Enter` or or you can just press `Ctrl + Alt + B`.

If it finishes successfully press `F1` again, type `PlatformIO: Upload` and select the corresponding option, or you can just press `Ctrl + Alt + U`.

### Run the nodes 
When the upload finishes you should remove the USB cable from the GCS. 

Start microBot docker container, by running:
```
cd ~/Projects/microBot/microbot_docker
bash docker_create
bash docker_exec
```

From the same terminal that you were before, run:
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
``` 

This will start the micro-ROS Agent.

Open another terminal and run:
```
cd ~/Projects/microBot/microbot_docker
bash docker_exec
```

Once you're inside the container, run:
```
ros2 topic list
``` 

Should you see something like:
```
/parameter_events
/rosout
/wheel_velocity
```
Then everything went well