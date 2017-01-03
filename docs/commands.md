# Networking Tools

	ip link
	sudo tcpdump -i eth0 -n

	sudo lshw -class networklspci | grep -i Ethernet
	sudo ifconfig -a | grep eth
	lspci | grep -i Ethernet

Useful to install:
- avahi-daemon (reference by hostname)

Some places to ping:
-nzchswlinxci.au.ivc
-kvw.local
-au.ivc


Addd global proxy:

    echo "http_proxy=PROXY; export http_proxy" | sudo tee ~/.bashrc


# X11 stuff
list opened connections

	watch netstat -tn

forward X windows over ssh with -X

	ssh -X user@host.local
