from setuptools import setup, find_packages

setup(
    name='myur',
    version='0.1.0',
    packages=find_packages(),
    install_requires=[
        'rclpy',
        'setuptools',
        'scipy',
        'numpy',
        'plotly'
    ],
    # entry_points={
    #     'console_scripts': [
    #         'my_action_client = my_package.my_action_client:main',  # Replace with actual module and function
    #     ],
    # },
    author='Tufts University CEEO',
    author_email='you@example.com',
    description='ROS2 Python package for Universal Robots',
    url='https://github.com/tuftsceeo/Universal-Robots-ROS2-CEEO',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.0',
)
