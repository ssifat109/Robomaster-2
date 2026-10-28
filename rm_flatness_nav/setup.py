from glob import glob
from setuptools import setup

setup(name='rm_flatness_nav', version='0.1.0', packages=['rm_flatness_nav'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/rm_flatness_nav']),
                  ('share/rm_flatness_nav', ['package.xml']),
                  ('share/rm_flatness_nav/launch', glob('launch/*.py')),
                  ('share/rm_flatness_nav/config', glob('config/*.yaml'))],
      install_requires=['setuptools', 'numpy'], zip_safe=True,
      entry_points={'console_scripts': [
          'planner = rm_flatness_nav.planner:main',
          'driver = rm_flatness_nav.driver:main',
          'logger = rm_flatness_nav.logger:main',
          'simulator = rm_flatness_nav.simulator:main']})
