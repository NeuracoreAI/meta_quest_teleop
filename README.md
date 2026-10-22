# Meta Quest Reader

Note this repo is an extension of: https://github.com/rail-berkeley/oculus_reader

This repository provides the tool to read the position and pressed button from the Meta Quest device.

Meta Quest reader consists of two elements: python script which receives the readings from the APK and the APK itself. **We use our pointer APK on the device** (see `src/axon_meta_quest/APK/`). Currently the pose of the controllers and pressed buttons are transferred from the APK. This behavior can be extended using provided APK [source code](app_source).

**Installing the APK on the device:** Follow the APK installation instructions from the [RAIL original repo](https://github.com/rail-berkeley/oculus_reader).

## Coordinate Systems: ROS vs OpenXR

This library works with two different coordinate systems that are important to understand:

### OpenXR Coordinate System
The Meta Quest device natively uses the **OpenXR coordinate system**:
- **X-axis**: Points to the right
- **Y-axis**: Points up
- **Z-axis**: Points backward (away from the user)

This is the coordinate system used internally by the Meta Quest tracking system and is the frame used by the poses published in `QuestHandsMsg`.

### ROS Coordinate System
ROS (Robot Operating System) uses a different convention:
- **X-axis**: Points forward
- **Y-axis**: Points left
- **Z-axis**: Points up

This is the standard coordinate system used in ROS and is available through `axon_meta_quest.transforms.openxr_to_ros()`.

### Conversion Between Systems
The conversion from OpenXR to ROS coordinates is performed using a static rotation quaternion `[0.5, -0.5, -0.5, 0.5]`. This transformation:
- Maps OpenXR right (+X) to ROS right (−Y)
- Maps OpenXR up (+Y) to ROS up (+Z)
- Maps OpenXR backward (+Z) to ROS backward (−X)

### Usage in Code
- **For ROS integration**: Use `axon_meta_quest.transforms.openxr_to_ros(matrix)` to convert a 4×4 OpenXR transform to ROS coordinates
- **For OpenXR/native data**: Use the poses published in `QuestHandsMsg`, which are in the native OpenXR coordinate system

When working with transforms, always be aware of which coordinate system you're using. See the docstrings in the code for specific coordinate system information for each function.

## Clone the repository

To pull the APK correctly, Git LFS has to be configured before cloning the repository. The installation is described here https://git-lfs.github.com. On Ubuntu follow these steps:

```bash
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs
git lfs install # has to be run only once on a single user account
```

Now you can clone this repository either with HTTPS or SSH.

## Install

Requires Python 3.12+ and `adb` on PATH (see [Setup of the ADB](#setup-of-the-adb)).
Python dependencies include NumPy, SciPy, PyYAML, and [neuracore_axon](https://github.com/NeuracoreAI/neuracore_axon)
(a private repo, so your git credentials need access to it).

### As a library (uv)

Add it to your project's `pyproject.toml`. The APKs are stored in Git LFS, so
`lfs = true` is required; without it you get pointer files instead of the APKs:

```toml
[project]
dependencies = ["axon_meta_quest"]

[tool.uv.sources]
axon_meta_quest = { git = "https://github.com/NeuracoreAI/meta_quest_teleop.git", branch = "main", lfs = true }
```

Then run `uv sync`. For a one-off install: `UV_GIT_LFS=1 uv pip install "axon_meta_quest @ git+https://github.com/NeuracoreAI/meta_quest_teleop.git"`.

The import name is `axon_meta_quest`:

```python
from axon_meta_quest import MetaQuestNode
```

The distribution and import name are now `axon_meta_quest`. Update older
`meta_quest` or `meta_quest_teleop` imports when upgrading.

### For development

```bash
uv sync          # creates .venv with the tested versions from uv.lock
uv run pytest
```

## Setup of the ADB

[ADB](https://developer.android.com/studio/command-line/adb) is required for the communication between Oculus Quest and the Python node.

To install ADB on Ubuntu run:
```
sudo apt install android-tools-adb
```

for mac, run:
```
brew install android-platform-tools
```

<details>
<summary>Instructions for new Meta Quest Device (Only run once after purchasing the Quest)</summary>

1. Determine your Meta Quest account name:
If you haven’t used Meta Quest before, start it and follow the steps to create your profile and get yourself started. Otherwise follow these steps to find out your username:
    1. Go to: [https://www.oculus.com/](https://www.oculus.com/) 
    2. Log in to account:
    ![image_0](https://user-images.githubusercontent.com/14967831/106832581-c7288f00-6646-11eb-91e0-3b74e81a58ba.png)
    3. After logging in **select your profile again** in top right corner and select **‘Profile’**
    ![image_1](https://user-images.githubusercontent.com/14967831/106832585-c859bc00-6646-11eb-9a3d-3a55f844ee37.png)
    4. You will be able to see your username on the following screen:
    ![image_2](https://user-images.githubusercontent.com/14967831/106832678-f7702d80-6646-11eb-823e-1001d6bffe01.png)
2. Enable Meta Quest development mode:
    1. If you belong to RAIL, inform me (Jedrzej Orbik) that you need to join the development organization. This is required to activate debugging mode on the device. Otherwise create your own organization <https://developer.oculus.com/manage/organizations/create/> and fill in the appropriate information.
    2. Turn on the device you want to use for development.
    3. Open the Meta app on your phone and then go to **Settings**.
    4. Tap the device and then go to **More Settings** > **Developer Mode**.
    5. Turn on the **Developer Mode** toggle.
    6. Connect your device to your computer using a USB-C cable and then wear the device.
    7. Accept **Allow USB Debugging** and **Always allow from this computer** when prompted to on the device.  
        ![image_3](https://user-images.githubusercontent.com/14967831/104061507-048d2e80-51f9-11eb-8327-7917f6a1ab60.png)  

</details>

## How to run the code

### Communication using the USB cable (easier to set up)

1. Connect Oculus Quest to PC with USB cable. This is required to establish the connection.
2. Run the example, which prints both controllers' state: `uv run python examples/meta_quest_example.py`

### Communication over the network (more portable)

1. Make sure that Oculus Quest is connected to the same network as the computer.
2. Connect Oculus Quest to PC with USB cable. This is required to establish the connection.
3. Put on the headset and allow the permission as requested.
4. Verify that a device is visible with: `adb devices`. The expected output:  
`List of devices attached`  
`    ce0551e7                device`
5. Check the IP address of the headset:  
    `adb shell ip route`  
    Expected output:  
    `10.0.30.0/19 dev wlan0  proto kernel  scope link  **src **10.0.32.101`
6. Read the IP address of the device standing after `**src`.
7. Provide the IP address when starting the node: `MetaQuestNode.start(ip_address="10.0.32.101")`, or run the example: `uv run python examples/meta_quest_example.py --ip 10.0.32.101`

### Stopping the app from adb
It is possible to stop the app from adb. Use: `adb shell am force-stop com.rail.oculus.teleop`

## AXOn node

`axon_meta_quest.MetaQuestNode` connects to the headset over adb, installs and launches
the teleop app, and runs as a
[neuracore_axon](https://github.com/NeuracoreAI/neuracore_axon) node. It is named
`meta_quest` and publishes a `QuestHandsMsg` (`axon_meta_quest.messages`) with both
controllers' OpenXR pose and buttons on `/meta_quest/hands`:

```python
from axon.runtime import RuntimeManager
from axon_meta_quest import MetaQuestNode

quest = MetaQuestNode.start()  # ip_address="..." for wireless
...
quest.terminate()
RuntimeManager.shutdown()
```

Subscribe from other nodes with
`self.create_subscriber("/meta_quest/hands", QuestHandsMsg, callback)`. The node
stops itself after 5 s without new controller data (`disconnect_timeout=None`
disables this). See `examples/meta_quest_example.py` for a runnable launcher.
