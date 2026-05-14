

This dumps every udpport signal and whows who sendt and who resieved during a 3 second interval (first is sender and second is reciever):
    sudo tshark -i lo -f "udp" -a duration:3 -T fields -e udp.srcport -e udp.dstport | sort -u



From inside the Ardupilot folder to start it (if other stuf fails):
    rm -rf build/px4_sitl_default
    make px4_sitl gz_x500