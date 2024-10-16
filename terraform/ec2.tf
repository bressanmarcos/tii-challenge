provider "aws" {
  region = "us-west-1"
}

# get latest ubuntu ami
data "aws_ami" "ubuntu" {
  most_recent = true

  filter {
    name   = "name"
    values = ["ubuntu/images/hvm-ssd/ubuntu-*-20.04-amd64-server-*"]
  }
}

# use official VPC module
module "vpc" {
  source  = "terraform-aws-modules/vpc/aws"
  version = "3.19.0"

  name           = "jenkins-vpc"
  cidr           = "10.0.0.0/16"
  azs            = ["us-west-1a"]
  public_subnets = ["10.0.1.0/24"]
}

resource "aws_security_group" "this" {
  name        = "jenkins and gazebo"
  description = "A not so secure security group for jenkins and gazebo instance"
  vpc_id      = module.vpc.vpc_id

  ingress {
    from_port   = 22
    to_port     = 22
    protocol    = "tcp"
    cidr_blocks = ["0.0.0.0/0"]
    description = "ssh"
  }

  ingress {
    from_port   = 5901
    to_port     = 5901
    protocol    = "tcp"
    cidr_blocks = ["0.0.0.0/0"]
    description = "vnc"
  }

  ingress {
    from_port   = 80
    to_port     = 80
    protocol    = "tcp"
    cidr_blocks = ["0.0.0.0/0"]
    description = "jenkins webserver"
  }
}

locals {
  instance_ip = "10.0.1.10"
}

resource "aws_instance" "this" {
  ami                    = data.aws_ami.ubuntu.id
  instance_type          = "t3a.medium"
  vpc_security_group_ids = [aws_security_group.this.id]
  subnet_id              = module.vpc.public_subnets[0]
  private_ip             = local.instance_ip

  tags = {
    Name = "jenkins and gazebo"
  }

  user_data = <<-USER_DATA
    #!/bin/bash

    # Install XFCE and TurboVNC
    apt-get update -y
    echo "set shared/default-x-display-manager lightdm" | debconf-communicate
    apt-get install -y xfce4 xfce4-goodies lightdm

    # Download and install TurboVNC
    wget https://sourceforge.net/projects/turbovnc/files/3.0.2/turbovnc_3.0.2_amd64.deb/download -O turbovnc.deb
    dpkg -i turbovnc.deb
    apt-get install -f -y  # Install any dependencies

    # Set up VNC password for the ubuntu user
    su - ubuntu -c 'mkdir -p ~/.vnc && echo -n "ubuntu" | vncpasswd -f > ~/.vnc/passwd'
    chmod 600 /home/ubuntu/.vnc/passwd

    # Create a VNC startup script for the ubuntu user
    su - ubuntu -c 'cat <<EOF > ~/.vnc/xstartup
    #!/bin/sh
    unset SESSION_MANAGER
    unset DBUS_SESSION_BUS_ADDRESS
    startxfce4 &
    EOF'
    chmod +x /home/ubuntu/.vnc/xstartup

    # Start the TurboVNC server as ubuntu user at boot
    cat <<EOF > /etc/systemd/system/vncserver@.service
    [Unit]
    Description=Start TurboVNC server at startup
    After=syslog.target network.target

    [Service]
    Type=forking
    User=ubuntu
    PAMName=login
    PIDFile=/home/ubuntu/.vnc/%H:%i.pid
    ExecStartPre=-/opt/TurboVNC/bin/vncserver -kill :%i > /dev/null 2>&1
    ExecStart=/opt/TurboVNC/bin/vncserver :%i
    ExecStop=/opt/TurboVNC/bin/vncserver -kill :%i

    [Install]
    WantedBy=multi-user.target
    EOF
    systemctl daemon-reload
    systemctl enable vncserver@1.service
    su - ubuntu -c "/opt/TurboVNC/bin/vncserver :1"

    # Allow VNC through the firewall (if necessary)
    ufw allow 5901/tcp

    # install docker
    curl -fsSL https://get.docker.com | sh
    usermod -aG docker ubuntu
    systemctl enable --now docker

    # change ubuntu password to "ubuntu"
    echo -n "ubuntu:ubuntu" | chpasswd

    # enable ssh access to ubuntu user using password authentication
    sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config
    sed -i 's/PasswordAuthentication no/PasswordAuthentication yes/' /etc/ssh/sshd_config
    systemctl restart ssh

    # add repo, install ros noetic and gazebo
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
    apt-get update -y
    apt-get install -y ros-noetic-desktop-full
    rosdep init
    su - ubuntu -c "rosdep update"

    # Add SSH key to authorized_keys for ubuntu user
    su - ubuntu -c 'mkdir -p ~/.ssh && echo "ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAACAQCeVsfidi90fu5/qkwJZfl0OtGksEUWV3EoYjnZeP7bp1SUByB2mnZioCClzlGIATSEICuInl/2l6wmG6Y6yolHa1bkP8UVPFpLuTAVtVN4kjuK9Rwcqa32tvucnfqtoWc8lb3ujymyOnHW4i8A66npu9HWXrGR4aimwZyMfa1Qbvoylf5cf0Lavy2OaKQ23jBtrWd6FDPShsWYSJYGXwpiRaIG41GGYOPJ92Fd2oJmqf8txFcdAQj1eOZEzY1PsHKG47oq0webnWyQhmDkY7YqlnpIlpHLIG+He52eLzegBBNhCGR3dhtCqe6e01OgP+Tc+PBAcQ1KBpYa0L3V0qJ8o+Cq4DLlp5glnrZezfBpQmo/ixiffszUypiMap0bjV3Juc3jROgBsqWZvKWf9CmkC/y98okXa8ITlOdBoFV3Y4W+3EiiPmQm0Tq97HHnORXNEHTjtYGPlKPoD14h2GWO3Y7iO/gDvE8Hostp857JleO394HnUEHk9sPWiLvOWDLO8vJh4qKRpkwL2LHEc5+g173tY/KrcyG5BPPY3Hk4KN4KN/8fDgAqe/UnM+Pp7G0XtuGwSjFJBPI4OLGDKyVe6rht8fjZTfgluPh+acsTiq6ZMOVuGTG+yWaeF9gKEBzn4aNNwACXtHaTvYlbT1Pelu2RmfGqdiHticv3KXu2ww== bressan@dee.ufc.br" >> ~/.ssh/authorized_keys'
    chmod 600 /home/ubuntu/.ssh/authorized_keys

    # Setup Jenkins
    su - ubuntu -c 'bash -s' <<EOF
        newgrp docker
        cd ~
        git clone https://github.com/bressanmarcos/tii-challenge.git
        cd tii-challenge/jenkins
        export GAZEBO_VM_USER=ubuntu
        export GAZEBO_VM_HOST=${local.instance_ip}
        export REMOTE_SIM_DIR=/home/ubuntu/tii-challenge
        docker compose up -d
    EOF

    USER_DATA
}

/*


*/
