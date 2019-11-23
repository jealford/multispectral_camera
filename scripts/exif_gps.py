import os
from fractions import Fraction
import piexif

def convert_to_deg(value, cardinal):
    if value < 0:
        cardinal_value = cardinal[0]
    elif value > 0:
        cardinal_value = cardinal[1]
    else:
        cardinal_value = ""

    degree = int(abs(value))
    minute = int((abs(value) - degree) * 60)
    second = round((((abs(value) - degree) * 60) - minute) * 60, 5)

    return (degree, minute, second, cardinal_value)

def convert_to_rational(number):
    f = Fraction(str(number))
    return (f.numerator, f.denominator)

def set_gps_location(image, lat, lng, altd):
    lat_deg = convert_to_deg(lat, ["S", "N"])
    lng_deg = convert_to_deg(lng, ["W", "E"])
    exiv_lat = (convert_to_rational(lat_deg[0]), convert_to_rational(lat_deg[1]), convert_to_rational(lat_deg[2]))
    exiv_lng = (convert_to_rational(lng_deg[0]), convert_to_rational(lng_deg[1]), convert_to_rational(lng_deg[2]))

    gps_ifd = {
        piexif.GPSIFD.GPSVersionID: (2, 0, 0, 0),
        piexif.GPSIFD.GPSAltitudeRef: 1,
        piexif.GPSIFD.GPSAltitude: convert_to_rational(round(altd)),
        piexif.GPSIFD.GPSLatitudeRef: lat_deg[3],
        piexif.GPSIFD.GPSLatitude: exiv_lat,
        piexif.GPSIFD.GPSLongitudeRef: lng_deg[3],
        piexif.GPSIFD.GPSLongitude: exiv_lng,
    }

    gps_exif = {"GPS": gps_ifd}

    # get original exif data first
    exif_data = piexif.load(image)

    # update original exif data to include GPS tag
    exif_data.update(gps_exif)
    exif_bytes = piexif.dump(exif_data)

    piexif.insert(exif_bytes, image)
