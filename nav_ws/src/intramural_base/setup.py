from setuptools import setup

setup(name="intramural_base", version="0.1.0", packages=["intramural_base"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/intramural_base"]),
                  ("share/intramural_base", ["package.xml", "LICENSE"])],
      install_requires=["setuptools"], zip_safe=True,
      maintainer="SRM intramural participant", maintainer_email="local@example.invalid",
      description="SRM navigation serial transport and explicit MCU simulator",
      license="Apache-2.0", entry_points={"console_scripts": [
          "serial_bridge = intramural_base.serial_bridge:main",
          "sim_mcu = intramural_base.sim_mcu:main"]})
