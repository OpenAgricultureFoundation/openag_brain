Compiling and Publishing a new Docker Image
-------------------------------------------

Updated docker images will be published by the core team from the openag_brain repository.

"Official" OpenAg Docker images are published by the core team from the openag_brain repository. If you want to contribute to a release, issue a `pull request <https://github.com/OpenAgInitiative/openag_brain/compare>`_. If you want to publish Docker images under your own account, you can follow these steps, substituting your own Docker info:

First, build the image:

    docker build -t openag/rpi_brain

Then, log in to docker:

    docker login

Finally, push the image to [Docker Hub](https://hub.docker.com/):

    docker push openag/rpi_brain