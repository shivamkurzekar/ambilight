# ===================================================================
#  Local settings for the Ambilight sender.
#
#  1. Copy this file to "config_local.py" (same folder).
#  2. Fill in your own values.
#
#  config_local.py is gitignored, so your local network details stay private.
#  Any value left out falls back to the default in ambilight_sender.py.
# ===================================================================

# UDP multicast group/port the ESP32 listens on
# (must match MCAST_IP / UDP_PORT in firmware/ambilight_receiver/secrets.h)
MCAST_GRP = "239.1.1.1"
PORT = 7777

# WiZ bulb (only used when ENABLE_WIZ_wiz = True in ambilight_sender.py)
BULB_IP = "192.168.0.100"
BULB_PORT = 38899
