# Path to save the CSV file
csv_file_path = 'overtap_output.csv'  # Adjust the path as needed for your environment

TARGET_ON = 3000
interval = 3000
ACCEPTABLE_HEAT = 100000
HOT_THRESHOLD = 150000
ATTENUATION_DENOMINATOR = 25
COOLING_DENOMINATOR = 100

attenuation = 0
onDurationUS = 0
lastheat = 0
heat = 0

# Open the file in write mode
with open(csv_file_path, 'w') as file:
    # Write the header
    file.write('Attenuation,On,Heat\n')
    
    for i in range(1, 201):
        heat = lastheat - interval//COOLING_DENOMINATOR
        if heat > ACCEPTABLE_HEAT:
            attenuation = (heat - ACCEPTABLE_HEAT) // ATTENUATION_DENOMINATOR
            onDurationUS = round(TARGET_ON - attenuation - TARGET_ON//ATTENUATION_DENOMINATOR)
        else:
            onDurationUS = TARGET_ON

        lastheat = heat + onDurationUS


        file.write(f'{attenuation},{onDurationUS},{heat}\n')
