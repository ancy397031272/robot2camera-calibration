from setuptools import setup


def readme():
    with open('README.rst') as f:
        return f.read()

def license():
    with open('license.rst') as f:
        return f.read()

setup(name='robot2cam_calibration',
      version='0.1',
      description='A package to calibrate a UR CB2 Robot to a camera',
      long_description=readme(),
      classifiers=[
        'Development Status :: 3 - Alpha',
        'Environment :: Console',
        'Intended Audience :: Manufacturing',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: MIT License',
        'Natural Language :: English',
        'Operating System :: OS Independent'
      ],
      keywords='universal robots cb2 ur5 flycapture2 point grey',
      url='-',
      author='Michael Sobrepera',
      author_email='mjsobrep@live.com',
      license=license(),
      packages=['robot2cam_calibration'],
      install_requires=[
          'numpy',
          'cv2',
          'ur_cb2'
      ],
      # dependency_links=['https://github.com/jordens/pyflycapture2'],
      include_package_data=True,
      entry_points={
        'console_scripts': [
            'robot2cam-record=robot2cam_calibration.get_correspondences:main']
      },
      zip_safe=False)
