

This dumps every udpport signal and whows who sendt and who resieved during a 3 second interval:
    sudo tshark -i lo -f "udp" -a duration:3 -T fields -e udp.srcport -e udp.dstport | sort -u

