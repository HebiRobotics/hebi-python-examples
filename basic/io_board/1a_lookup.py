import hebi
from time import sleep

lookup = hebi.Lookup()

# Wait 2 seconds for the module list to populate, and then print out its contents
sleep(2.0)

for entry in lookup.entrylist:
  print(entry)

print('NOTE:')
print('  The listing above should show the information for all the modules')
print('  on the local network. If this is empty make sure that the modules')
print('  are connected, powered on, and that the status LEDs are displaying')
print('  a green soft-fade.')



