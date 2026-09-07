from setuptools import setup

package_name = "spine_ht_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=["spine_ht_ros"],
    package_dir={"": "."},
    data_files=[
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zac",
    maintainer_email="your@email.com",
    description="Hybrid ROS2 package",
    license="Apache 2.0",
    entry_points={
        "console_scripts": [
            "spine_node = spine_ht_ros.spine_node:main",
            "twist_converter = spine_ht_ros.twist_converter:main",
        ],
    },
)
