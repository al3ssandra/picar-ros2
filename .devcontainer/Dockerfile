FROM ros:iron-ros-base-jammy
# ARG USERNAME=USERNAME

# # Create the user
# RUN groupadd --gid $USER_GID $USERNAME \
#     && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
#     #
#     # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
#     && apt-get update \
#     && apt-get install -y sudo \
#     && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
#     && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y && apt-get install \
    locales

RUN locale-gen en_GB en_GB.UTF-8
RUN update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8

ENV LANG=en_GB.UTF-8

RUN apt-get install -y python3-pip

ENV SHELL /bin/bash

# [Optional] Set the default user. Omit if you want to keep the default as root.
# USER $USERNAME
# CMD ["/bin/bash"]

ENTRYPOINT ["tail"]
CMD ["-f","/dev/null"]