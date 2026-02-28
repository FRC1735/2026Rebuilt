import hid

for d in hid.enumerate():
    print(d['vendor_id'], d['product_id'], d['product_string'])