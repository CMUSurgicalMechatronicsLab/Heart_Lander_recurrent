from Load_cell_voltage_scan import *
import numpy as np

def main():
    Cerb_load_cell = Load_Cell()

    Cerb_load_cell.initialize_daq()

    anterior_mean = 0
    posterior_mean = 0

    anterior_threshold = 0.5  # just arbitrary voltage for testing

    while anterior_mean < anterior_threshold:
        anterior_data, posterior_data = Cerb_load_cell.get_continuous_reading()
        anterior_mean = np.round(np.mean(anterior_data), 3)
        posterior_mean = np.round(np.mean(posterior_data), 3)


if __name__ == '__main__':
    main()