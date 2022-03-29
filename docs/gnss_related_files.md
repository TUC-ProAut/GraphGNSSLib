# Getting GNSS related files

Multiple files can or must be added to the GNSS observations received from the multi-constellation GNSS receiver. Different websites are used for different products.

You may have to double-check if the downloaded data actually contains the desired information.

Please make sure that you include the downloaded files in your dataset folder and specify the correct path inside the launchfile.


## 1. Ephemeris

The ephemerides of the used satellite systems must be added for the [RTKLIB](http://www.rtklib.com/) to be able to compute pseudoranges. Two different types of files can be used:

### 1.1 Seperated files for the satellite-systems

Every satellite-system has its own ephemeride file. These can be downloaded from the CDDIS (Crustal Dynamics Data Information System). To be able to download files you need to register for NASA's EARTHDATA login. This is free of charge.
The data is divided into GPS weeks and GPS days. [This website](https://www.ngs.noaa.gov/CORS/Gpscal.shtml) can be used to determine this information.
The data is provided by multiple GNSS stations around the world (encoded in the file name: 4-character IGS station name or 4-character site monument name). An overview of all stations can be found here: [https://igs.org/network/](https://igs.org/network/).
The following website describes the logic behind the naming of the files and folders: [https://cddis.nasa.gov/Data_and_Derived_Products/GNSS/daily_30second_data.html](https://cddis.nasa.gov/Data_and_Derived_Products/GNSS/daily_30second_data.html) (click View format codes and data sets either for RINEX 2 or RINEX 3).
The data can be downloaded here: [https://cddis.nasa.gov/archive/gnss/data/daily/](https://cddis.nasa.gov/archive/gnss/data/daily/) .
When using the separate files you have to set *shared_ephemeris* in the config file to **false**.

### 1.2 Combined file (shared epehermis)

Other than using separate files for each satellite system you can also download one file containing all systems of interest (if available).
Most likely you will have to get the **daily archive** (BRDC00WRD _... files) from here: [https://igs.org/mgex/data-products/#bce](https://igs.org/mgex/data-products/#bce) .
When using the one combined file you have to set *shared_ephemeris* in the config file to **true**.

## 2. Precise ephemeris

SP3-files contain the precise satellite orbit solution of GNSS satellites. The data can be downloaded from here: [https://igs.org/mgex/data-products/#orbit_clock](https://igs.org/mgex/data-products/#orbit_clock). Further information regarding the file naming logic and different places to download the files can also be found there. An EARTHDATA login will probably be needed (see section 1. Ephemeris).

Please make sure, that all desired satellite systems are present in the downloaded file. Otherwise, the missing systems will not be used by the RTKLIB.

When using the precise ephemeris you have to set *precise_ephemeris* in the config file to **true**.

## 3. Ionosphere correction

Accurate ionospheric correction can be downloaded from here: [https://cddis.nasa.gov/archive/gnss/products/ionex/](https://cddis.nasa.gov/archive/gnss/products/ionex/) (EARTHDATA login required, see section 1. Ephemeris). Further information: [https://cddis.nasa.gov/Data_and_Derived_Products/GNSS/atmospheric_products.html#iono](https://cddis.nasa.gov/Data_and_Derived_Products/GNSS/atmospheric_products.html#iono).

These files contain the ionosphere vertical total electron content (TEC) maps provided in the IONEX format.

When using the IONEX files you have to set *ionex_correction* in the config file to **true**.

## 4. Custom antenna model

The current IGS antenna phase center model can be downloaded here: [https://igs.org/mgex/data-products/#orbit_clock](https://igs.org/mgex/data-products/#orbit_clock).

When using the antenna model file you have to set *custom_atx* in the config file to **true**.
