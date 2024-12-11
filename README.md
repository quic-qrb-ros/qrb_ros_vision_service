# QRB ROS Vision Service

## Overview

`qrb_ros_vision_service` is a ROS2 package designed to help developers quickly develop vision feature on the QRB platform.
It assists developers in rapidly setting up vision-related pipelines and currently supports functions
such as object detection, camera, video processing, and QR code recognition.

## Features
- **Task type**: This type can help developers set up the corresponding pipeline and return the output topic of the last node to the app. In this type, we support functions such as camera, video, and object detection.
- **Case type**: This type is designed for developers who want to directly obtain task results. It can continuously return task results until the user manually stops it. Currently, it supports QR code detection.
- **Provide configuration files**: Provide configuration files to modify the parameters of related nodes.

## Contributing

We would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](./CONTRIBUTING.md) and [code of conduct](./CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](../../issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](../../issues)


## Authors

* **Zhanye Lin** - *Initial work* - [zhanlin](https://github.com/quic-zhanlin)

See also the list of [contributors](https://github.com/quic-qrb-ros/qrb_ros_vision_service/graphs/contributors) who participated in this project.


## License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.
