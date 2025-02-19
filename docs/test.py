import os

content = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Hello World</title>
</head>
<body>
    <h1>Hello world</h1>
</body>
</html>
"""

out_dir = "build"
if not os.path.isdir(out_dir):
    os.makedirs(out_dir)

with open("build/index.html") as f:
    f.write(content)
