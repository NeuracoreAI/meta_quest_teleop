"""Setup script for the Meta Quest teleop package."""

from setuptools import setup

setup(
    name="meta_quest_teleop",
    version="1.0.0",
    packages=["meta_quest_teleop"],
    license="Apache-2.0 License",
    long_description=open("README.md").read(),
    install_requires=["numpy", "pure-python-adb"],
    package_data={
        "": ["APK/teleop-pointer-frame-relative.apk", "APK/teleop-rail-orig.apk"]
    },
    include_package_data=True,
)
