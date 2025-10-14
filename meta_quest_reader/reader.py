"""Meta Quest Reader."""

import os
import sys
import threading
import time
from typing import Any

import numpy as np
from ppadb.client import Client as AdbClient

from meta_quest_reader.buttons_parser import parse_buttons
from meta_quest_reader.FPS_counter import FPSCounter


def eprint(*args: Any, **kwargs: Any) -> None:
    """Print error messages to stderr."""
    RED = "\033[1;31m"
    sys.stderr.write(RED)
    print(*args, file=sys.stderr, **kwargs)
    RESET = "\033[0;0m"
    sys.stderr.write(RESET)


class MetaQuestReader:
    """Meta Quest Reader class."""

    def __init__(
        self,
        ip_address: str | None = None,
        port: int = 5555,
        APK_name: str = "com.rail.oculus.teleop",
        print_FPS: bool = False,
        run: bool = True,
    ) -> None:
        """Init.

        Args:
            ip_address: IP address of the Meta Quest device.
                If None, USB connection is used.
            port: Port for the ADB connection. Defaults to 5555.
            APK_name: Name of the APK to be installed on the Meta Quest device.
                Defaults to "com.rail.oculus.teleop".
            print_FPS: Whether to print the FPS. Defaults to False.
            run: Whether to start reading data immediately. Defaults to True.
        """
        self.running = False
        self.last_transforms: dict[str, Any] | None = {}
        self.last_buttons: dict[str, Any] | None = {}
        self._lock = threading.Lock()
        self.tag = "wE9ryARX"

        self.ip_address = ip_address
        self.port = port
        self.APK_name = APK_name
        self.print_FPS = print_FPS
        if self.print_FPS:
            self.fps_counter = FPSCounter()

        self.device = self.get_device()
        self.install(verbose=False)
        if run:
            self.run()

    def __del__(self) -> None:
        """Destructor."""
        self.stop()

    def run(self) -> None:
        """Start reading data from the Meta Quest device."""
        self.running = True
        self.device.shell(
            'am start -n "com.rail.oculus.teleop/com.rail.oculus.teleop.MainActivity" '
            "-a android.intent.action.MAIN -c android.intent.category.LAUNCHER"
        )
        self.thread = threading.Thread(
            target=self.device.shell, args=("logcat -T 0", self.read_logcat_by_line)
        )
        self.thread.start()

    def stop(self) -> None:
        """Stop reading data from the Meta Quest device."""
        self.running = False
        if hasattr(self, "thread"):
            self.thread.join()

    def get_network_device(self, client: AdbClient, retry: int = 0) -> Any:
        """Get the Meta Quest device over the network.

        Args:
            client: ADB client.
            retry: Retry count.

        Returns:
            The Meta Quest device.
        """
        try:
            client.remote_connect(self.ip_address, self.port)
        except RuntimeError:
            os.system("adb devices")
            client.remote_connect(self.ip_address, self.port)
        assert self.ip_address is not None
        device = client.device(self.ip_address + ":" + str(self.port))

        if device is None:
            if retry == 1:
                os.system("adb tcpip " + str(self.port))
            if retry == 2:
                eprint(
                    "Make sure that device is running and is available at the "
                    "IP address specified as the OculusReader argument `ip_address`."
                )
                eprint("Currently provided IP address:", self.ip_address)
                eprint("Run `adb shell ip route` to verify the IP address.")
                exit(1)
            else:
                self.get_device()
                raise RuntimeError("Could not connect to device.")
        return device

    def get_usb_device(self, client: AdbClient) -> Any:
        """Get the Meta Quest device over USB.

        Args:
            client: ADB client.

        Returns:
            The Meta Quest device.
        """
        try:
            devices = client.devices()
        except RuntimeError:
            os.system("adb devices")
            devices = client.devices()
        for device in devices:
            if device.serial.count(".") < 3:
                return device
        eprint(
            "Device not found. Make sure that device is running "
            "and is connected over USB"
        )
        eprint("Run `adb devices` to verify that the device is visible.")
        exit(1)

    def get_device(self) -> Any:
        """Get the Meta Quest device.

        Returns:
            The Meta Quest device.
        """
        # Default is "127.0.0.1" and 5037
        client = AdbClient(host="127.0.0.1", port=5037)
        if self.ip_address is not None:
            return self.get_network_device(client)
        else:
            return self.get_usb_device(client)

    def install(
        self, APK_path: str | None = None, verbose: bool = True, reinstall: bool = False
    ) -> None:
        """Install the APK on the Meta Quest device.

        Args:
            APK_path: Path to the APK file. If None, the default path is used.
            verbose: Whether to print messages. Defaults to True.
            reinstall: Whether to reinstall the APK if it is already installed.
                Defaults to False.
        """
        try:
            installed = self.device.is_installed(self.APK_name)
            if not installed or reinstall:
                if APK_path is None:
                    APK_path = os.path.join(
                        os.path.dirname(os.path.realpath(__file__)),
                        "APK",
                        "teleop-debug.apk",
                    )
                success = self.device.install(APK_path, test=True, reinstall=reinstall)
                installed = self.device.is_installed(self.APK_name)
                if installed and success:
                    print("APK installed successfully.")
                else:
                    eprint("APK install failed.")
            elif verbose:
                print("APK is already installed.")
        except RuntimeError:
            eprint("Device is visible but could not be accessed.")
            eprint(
                "Run `adb devices` to verify that the device is visible and accessible."
            )
            eprint(
                'If you see "no permissions" next to the device serial, '
                "please put on the Meta Quest and allow the access."
            )
            exit(1)

    def uninstall(self, verbose: bool = True) -> None:
        """Uninstall the APK from the Meta Quest device.

        Args:
            verbose: Whether to print messages. Defaults to True.
        """
        try:
            installed = self.device.is_installed(self.APK_name)
            if installed:
                success = self.device.uninstall(self.APK_name)
                installed = self.device.is_installed(self.APK_name)
                if not installed and success:
                    print("APK uninstall finished.")
                    print(
                        "Please verify if the app disappeared from the "
                        'list as described in "UNINSTALL.md".'
                    )
                    print(
                        "For the resolution of this issue, please follow "
                        "https://github.com/Swind/pure-python-adb/issues/71."
                    )
                else:
                    eprint("APK uninstall failed")
            elif verbose:
                print("APK is not installed.")
        except RuntimeError:
            eprint("Device is visible but could not be accessed.")
            eprint(
                "Run `adb devices` to verify that the device is visible and accessible."
            )
            eprint(
                'If you see "no permissions" next to the device serial, '
                "please put on the Oculus Quest and allow the access."
            )
            exit(1)

    @staticmethod
    def process_data(
        string: str,
    ) -> tuple[dict[str, np.ndarray] | None, dict[str, Any] | None]:
        """Process data from the Meta Quest device.

        Args:
            string: String to process.

        Returns:
            Tuple of transformations and button states.
        """
        try:
            transforms_string, buttons_string = string.split("&")
        except ValueError:
            return None, None
        split_transform_strings = transforms_string.split("|")
        transforms = {}
        for pair_string in split_transform_strings:
            transform = np.empty((4, 4))
            pair = pair_string.split(":")
            if len(pair) != 2:
                continue
            left_right_char = pair[0]  # is r or l
            transform_string = pair[1]
            values = transform_string.split(" ")
            c = 0
            r = 0
            count = 0
            for value in values:
                if not value:
                    continue
                transform[r][c] = float(value)
                c += 1
                if c >= 4:
                    c = 0
                    r += 1
                count += 1
            if count == 16:
                transforms[left_right_char] = transform
        buttons = parse_buttons(buttons_string)
        return transforms, buttons

    def extract_data(self, line: str) -> str:
        """Extract data from a line of logcat output.

        Args:
            line: Line of logcat output.

        Returns:
            Extracted data.
        """
        output = ""
        if self.tag in line:
            try:
                output += line.split(self.tag + ": ")[1]
            except ValueError:
                pass
        return output

    def get_transformations_and_buttons(
        self,
    ) -> tuple[dict[str, np.ndarray] | None, dict[str, Any] | None]:
        """Get the latest transformations and button states.

        Returns:
            Tuple of transformations and button states.
        """
        with self._lock:
            return self.last_transforms, self.last_buttons

    def read_logcat_by_line(self, connection: Any) -> None:
        """Read logcat output line by line.

        Args:
            connection: Connection to read from.
        """
        file_obj = connection.socket.makefile()
        while self.running:
            try:
                line = file_obj.readline().strip()
                data = self.extract_data(line)
                if data:
                    transforms, buttons = MetaQuestReader.process_data(data)
                    with self._lock:
                        self.last_transforms, self.last_buttons = transforms, buttons
                    if self.print_FPS:
                        self.fps_counter.get_and_print_fps()
            except UnicodeDecodeError:
                pass
        file_obj.close()
        connection.close()


def main() -> None:
    """Main function to test the MetaQuestReader."""
    oculus_reader = MetaQuestReader()

    while True:
        time.sleep(0.3)
        print(oculus_reader.get_transformations_and_buttons())


if __name__ == "__main__":
    main()
