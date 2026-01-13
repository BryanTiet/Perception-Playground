from setuptools import find_packages, setup

package_name = "perception_playground"

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
    maintainer="TODO",
    maintainer_email="todo@example.com",
    description="Nav2/Turtlebot3 playground workspace package.",
    license="TODO",
    tests_require=["pytest"],
)
