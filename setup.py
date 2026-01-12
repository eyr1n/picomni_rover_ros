from setuptools import find_packages, setup

package_name = "picomni_rover_ros"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rin Iwai",
    maintainer_email="me@eyrin.jp",
    description="TODO: Package description",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "picomni_rover_ros = picomni_rover_ros.picomni_rover_ros:main",
        ],
    },
)
