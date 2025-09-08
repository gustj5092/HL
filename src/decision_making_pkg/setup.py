# 파일 경로: src/decision_making_pkg/setup.py

from setuptools import setup

package_name = 'decision_making_pkg'

setup(
    name=package_name,
    version='0.0.0',
    # find_packages() 대신 패키지 경로를 명시적으로 지정하여 정확도를 높입니다.
    packages=[
        package_name,
        f'{package_name}.lib'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hhs',
    maintainer_email='hhs@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner_node = decision_making_pkg.path_planner_node:main',
            'motion_planner_node = decision_making_pkg.motion_planner_node:main',
            'path_planner_robust_node = decision_making_pkg.path_planner_robust_node:main',
        ],
    },
)