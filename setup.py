from setuptools import setup


def readme():
    with open('README.rst') as f:
        return f.read()

def license():
    with open('license.rst') as f:
        return f.read()

setup(name='robot2cam_calibration',
      version='0.1',
      description='A package to calibrate a robot arm to a camera',
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
      keywords='universal robots cb2 ur5 flycapture2 point grey kuka',
      url='https://github.com/IRIM-Technology-Transition-Lab/robot2camera-calibration',
      author='Michael Sobrepera',
      author_email='mjsobrep@live.com',
      license=license(),
      packages=['robot2cam_calibration'],
      install_requires=[
          'numpy',
          'cv2',
          'scipy'
      ],
      # dependency_links=['https://github.com/jordens/pyflycapture2'],
      include_package_data=True,
      entry_points={
        'console_scripts': [
            'robot2cam-record-ur=robot2cam_calibration.get_correspondences:main',
            'robot2cam-images-ur=robot2cam_calibration.get_images:main',
            'robot2cam-compute=robot2cam_calibration.compute_transformations:main',
            'robot2cam-check=robot2cam_calibration.check_transformation:main'
        ]
      },
      zip_safe=False)
