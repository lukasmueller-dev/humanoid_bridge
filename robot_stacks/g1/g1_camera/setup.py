from setuptools import setup

package_name = "g1_camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    # OpenCV is deliberately absent: on the Jetson it comes from JetPack and
    # there is no aarch64 wheel to fall back on.
    install_requires=["setuptools", "numpy", "pyzmq"],
    python_requires=">=3.8",
    zip_safe=True,
    maintainer="Lukas Mueller",
    maintainer_email="lukasbmueller@gmail.com",
    description="Camera server and client for the G1's onboard Jetson.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_server = g1_camera.server:main",
            "fake_camera_server = g1_camera.fake:main",
        ],
    },
)
