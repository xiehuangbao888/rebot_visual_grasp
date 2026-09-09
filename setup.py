from glob import glob

from setuptools import find_packages, setup

package_name = "rebot_visual_grasp"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    entry_points={
        "console_scripts": [
            "align_mount = rebot_visual_grasp.align_mount:main",
            "align_camera = rebot_visual_grasp.align_camera:main",
            "grasp_marker = rebot_visual_grasp.grasp_marker:main",
            "grasp_click = rebot_visual_grasp.grasp_click:main",
            "grasp_execute = rebot_visual_grasp.grasp_execute:main",
            "grasp_yolo = rebot_visual_grasp.grasp_yolo:main",
        ],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/scripts", glob("scripts/*")),
        (f"share/{package_name}/docs", glob("docs/*")),
        (f"share/{package_name}/config", glob("config/*")),
        (
            f"share/{package_name}/description/urdf",
            glob("description/urdf/*.urdf") + glob("description/urdf/*.xacro"),
        ),
        (
            f"share/{package_name}/description/meshes",
            glob("description/meshes/*"),
        ),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yuan",
    maintainer_email="support@example.com",
    description="Gemini 2 wrist camera model, mount alignment, and visual grasp tools.",
    license="Apache-2.0",
)
