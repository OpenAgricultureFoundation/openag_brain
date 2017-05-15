#!/bin/bash
curl -s https://packagecloud.io/install/repositories/Hypriot/Schatzkiste/script.deb.sh | sudo bash
sudo apt-get install -y docker-hypriot docker-compose
sudo usermod -aG docker pi
sudo systemctl enable docker.service
echo Finished installing docker. You may have to restart for all changes to take effect
