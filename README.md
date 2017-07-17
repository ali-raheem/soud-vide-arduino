# Unnamed Sous Vide PID controller

Yet another arduino based PID controller targeting the crowded sous vide market.

Untested WIP, I'm bumbling through this a bit, the probably works but will it work well? One issue is my TPC implementation if you have a PID which is making changes less than the TPC_WINDOW size frequency that will mean the TPC duty cycle will change immediately.
