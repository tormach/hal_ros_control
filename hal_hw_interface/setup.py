from setuptools import setup

package_name = "hal_hw_interface"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="John Morris",
    maintainer_email="john@zultron.com",
    description="Machinekit HAL interface to robot hardware and I/O",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scripts/hal_io",
            "scripts/hal_mgr",
        ],
    },
)
