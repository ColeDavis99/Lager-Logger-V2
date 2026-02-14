# Convert `dashboard.html` to a Clean Comma-Separated Decimal Block  
### Using the `awk` / `sed` Method (Git Bash Terminal Commands)
Only do these steps if you need to edit the html, css, or js of the webpage. These steps compress the dashboard.html file and output a comma-seperated decimal block to save flash memory on the ESP32 and reduce boot-up time when first serving the webpage.
Here are the three steps to go from `dashboard.html` to a clean, comma-separated decimal block with line breaks, using the `awk` / `sed` method to ensure your formatting is C++ ready.

---

## 1. Compress the HTML

This creates the `dashboard.html.gz` file at maximum compression.

```bash
gzip -k -9 -f dashboard.html
```

---

## 2. Get the Array Size

Run this to get the number for your `DASH_HTML[SIZE]` brackets in `edp.cpp` and `edp.h`:

```bash
wc -c < dashboard.html.gz
```

---

## 3. Generate the Clean Decimal Dump

This command ensures:

- Every number has a comma  
- Line breaks are preserved for readability  
- Numbers do not "merge" across lines  

```bash
od -An -v -t u1 dashboard.html.gz | awk '{$1=$1;print}' | sed 's/ /,/g; s/$/,/' > edp_array.txt
```

---

# Final Polish (Before You Compile)

When you open `new_array.txt` and paste it into your `edp.cpp` file:

## The Starting Line

Ensure your variable starts correctly:

```cpp
const uint8_t DASH_HTML[SIZE_FROM_STEP_2] PROGMEM = {
```

---

## The Very Last Number

Because the command adds a comma to the end of every line, your very last number will have a trailing comma (e.g., `..., 105,` followed by `};`).

---

## Action

Delete that last comma right before the `};` to keep the compiler happy.
