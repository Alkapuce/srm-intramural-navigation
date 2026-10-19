from setuptools import setup

setup(
    name="intramural_mission",
    version="0.1.0",
    packages=["intramural_mission"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/intramural_mission"]),
        ("share/intramural_mission", ["package.xml", "LICENSE"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SRM intramural participant",
    maintainer_email="local@example.invalid",
    description="Ordered figure-eight mission",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mission = intramural_mission.mission:main",
            "trajectory_mission = intramural_mission.trajectory_mission:main",
            "adapter = intramural_mission.adapter:main",
            "lidar_adapter = intramural_mission.lidar_adapter:main",
            "livox_adapter = intramural_mission.livox_adapter:main",
        ]
    },
)
