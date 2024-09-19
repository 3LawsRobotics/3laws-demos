from setuptools import setup

package_name = "lll_teleop_twist_keyboard"

setup(
    name=package_name,
    version="1.0.0",
    packages=[],
    py_modules=["lll_teleop_twist_keyboard"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Thomas Gurriet",
    maintainer_email="tgurriet@3laws.io",
    author="Thomas Gurriet",
    keywords=["ROS"],
    description="Teleop interface for two twist commanded robots - ROS2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["lll_teleop_twist_keyboard = lll_teleop_twist_keyboard:main"],
    },
)
