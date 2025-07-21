from setuptools import setup

package_name = 'ar_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ваше Имя',
    maintainer_email='you@example.com',
    description='Описание вашего пакета',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            # если у вас есть исполняемые скрипты, например:
            # 'talker = ar_project.node:main',
        ],
    },
)
