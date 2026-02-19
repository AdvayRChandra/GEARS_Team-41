# Contributing Guidelines

This document outlines the code standards for all Python contributions to this project, based on the ENGR 141/142 Code Standard (V4.1.3).

Where this guide incorporates conventions from [PEP 8](https://peps.python.org/pep-0008/) (the Python community style guide), the **ENGR 141/142 Code Standard always takes precedence** over PEP 8 on any conflicting point. Notable conflicts:

| Topic | Code Standard (authoritative) | PEP 8 (deferred) |
|---|---|---|
| Variable & function names | `camelCase` | `snake_case` |
| Max line length | 80 characters (break at 75) | 79 characters |
| Constants | `ALL_CAPS_WITH_UNDERSCORES` | Same (no conflict) |

PEP 8 conventions are adopted **only** for areas the Code Standard does not address (e.g., class naming, docstrings, module structure).

---

## Table of Contents

1. [File Header](#1-file-header)
2. [Python File Organization](#2-python-file-organization)
3. [Formatting](#3-formatting)
   - [Indentation](#indentation)
   - [Spacing](#spacing)
   - [Parentheses](#parentheses)
   - [Line Length & Splitting](#line-length--splitting)
4. [Robustness](#4-robustness)
   - [Error Handling](#error-handling)
   - [Global Variables](#global-variables)
5. [Style & Readability](#5-style--readability)
   - [Commenting](#commenting)
   - [Capitalization](#capitalization)
   - [Variable Naming](#variable-naming)
   - [Variable Input](#variable-input)
6. [Python-Specific Rules](#6-python-specific-rules)
   - [Imports](#imports)
   - [The `with` Statement](#the-with-statement)
   - [Reserved Keywords](#reserved-keywords)
7. [Modules & Classes (PEP 8 Supplement)](#7-modules--classes-pep-8-supplement)
   - [Module Organization](#module-organization)
   - [Class Naming](#class-naming)
   - [Docstrings](#docstrings)
   - [Module-Level Variables](#module-level-variables)
   - [`__init__.py` Files](#__init__py-files)
   - [Relative Imports](#relative-imports)

---

## 1. File Header

Every file must begin with a header formatted as comments using `#`. The header must include the activity name, filename, date, authors, section, team number, electronic signatures, and a brief description.

**Team assignment header:**

```python
# Activity X.X.X: An introduction to Python.
# File:    filename.py
# Date:    1 September 2014
# By:      Full Name team member 1
#          Login ID
#          Full Name team member 2
#          Login ID
#          Full Name team member 3
#          Login ID
#          Full Name team member 4
#          Login ID
# Section: Number
# Team:    Team Number
#
# ELECTRONIC SIGNATURE
# Full Name team member 1
# Full Name team member 2
# Full Name team member 3
# Full Name team member 4
#
# The electronic signatures above indicate that the program
# submitted for evaluation is the combined effort of all
# team members and that each member of the team was an
# equal participant in its creation.  In addition, each
# member of the team has a general understanding of
# all aspects of the program development and execution.
#
# A BRIEF DESCRIPTION OF WHAT THE PROGRAM OR FUNCTION DOES
```

---

## 2. Python File Organization

Python files must be organized in the following order:

1. Program header (see above)
2. Import statements
3. User-defined functions
4. Main program code

---

## 3. Formatting

### Indentation

- Indent control structures **4 spaces** from the enclosing block.
- **Do not use tabs.** Use the space bar only. Tab width varies across environments; spaces ensure consistency.

```python
# Good
for i in range(0, 5):
    sum = sum + i
```

### Spacing

- Place **one space** on each side of arithmetic and logical operators.

```python
# Good
newTemperature = initialTemperature * 50.0
myGrade = 70.0 + examScore * aScalar

# Bad
myGrade=70.0+examScore*aScalar
```

- Place **one space after commas** separating arguments or list elements.

```python
# Good
myFun(2, 1, 2)
```

- An optional space before a parameter list's opening parenthesis is permitted.

```python
# Both acceptable
def myFun(A, B, C):
def myFun (A, B, C):
```

### Parentheses

Each sub-expression in a compound logical expression must be parenthesized.

```python
if (examScore > 0) and (examScore <= 100):
```

### Line Length & Splitting

- Lines must **not exceed 80 characters**.
- Break at or before **75 characters** to provide a safety margin.
- When splitting long lines, align continuation arguments or use consistent hanging indentation:

```python
# Align with opening delimiter
var = function_call(one_Var, two_Var,
                    three_Var, four_Var)

# Or place all arguments on the next line with consistent indentation
func = function_call(
    one_Var, two_Var,
    three_Var, four_Var)

# Function definition split
def function_call(
        one_Var, two_Var, three_Var,
        four_Var):
```

> Backslash line continuation is not recommended. Prefer parentheses for implied continuation.

---

## 4. Robustness

### Error Handling

- Make code as robust and readable as possible to facilitate debugging.
- Do **not** wrap large portions of code in a single error-trapping structure. Use small, specific, well-commented blocks instead.
- Verify each logical step of your code before proceeding to the next.
- Test code thoroughly before submission using your own test scripts and test cases.

### Global Variables

- **Do not use global variables.** All variables must be defined within functions.
- Pass variables between functions as needed; do not rely on shared global state.
- Global variables reduce modularity and make it difficult to trace when values change.

---

## 5. Style & Readability

### Commenting

- Comment frequently to explain the purpose of blocks of code.
- Include units in comments for variables, constants, inputs, and outputs.
- Do **not** comment on obvious syntax — explain intent and purpose.

```python
# Good
# Initial temperature
temperature = 10  # degrees

# Bad
# Variable to store temperature values
temperature = 10
```

- If a block's purpose is not immediately obvious, it must have a comment.
- Always include a **blank line above** a comment line.
- For multi-line comment blocks, include a **blank line above and below** the block.
- Limit comment line length to **72 characters**.
- If you modify code, update the corresponding comments.

```python
# Good: multi-line comment block with blank lines
# If the score is valid
if ((i > 0) and (i <= 100)):

    # Increase the total grade and print the student's
    # score
    grade = grade + i
    print('Total Grade = ', grade)
```

### Capitalization

- Commands, user-defined functions, and reserved words must begin with a **lowercase letter**.
- **Constants** must be `ALL_CAPS_WITH_UNDERSCORES`.

```python
MAXIMUM_VALUE = 100
SEC_PER_MIN = 60
```

### Variable Naming

- Use **camelCase**: first word lowercase, each subsequent word capitalized.

```python
velocity
angularAcceleration
```

- Variable names must be **descriptive and meaningful**. Avoid unclear abbreviations.

```python
# Good
computeArrivalTime
travelDistance
numberOfCars

# Bad
comparr
dist
cNum
```

- Large-scope variables (used throughout a function) must have especially clear names.
- Small-scope variables (loop and matrix indices) may be less descriptive; use conventions like `index1`, `row1`, `col2`, or alphabetical letters for nested loops (`i`, `j`).
- Words in a name may optionally be separated by underscores (`myAge`, `my_age`). Names are case-sensitive.
- Variable names may be up to **31 characters** in length.
- Do not use Python reserved keywords as variable names (see [Reserved Keywords](#reserved-keywords)).

### Variable Input

- Always precede input prompts with a message describing the expected input.
- Follow the required input order specified by the assignment or interface contract.

---

## 6. Python-Specific Rules

### Imports

- All import statements must appear at the **top of the file**, immediately after the header.
- Each import must be on its **own line**.

```python
# Good
import sys
import math
from keyword import iskeyword as testWord

# Bad
import sys, math
```

### The `with` Statement

Use a `with` statement when opening files. It ensures files are closed automatically. A comment **must** be placed at the end of the `with` block to indicate that the file closes there.

```python
with open('textfile.txt', 'r') as f:  # This opens textfile.txt
    val1 = int(f.readline())
    val2 = int(f.readline())
    product = val1 * val2
    # textfile.txt closes here  <- This comment must be included

print("Product is %d" % product)
```

### Reserved Keywords

The following words are reserved in Python and **cannot** be used as identifiers:

| | | | | |
|---|---|---|---|---|
| `and` | `del` | `from` | `not` | `while` |
| `as` | `elif` | `global` | `or` | `with` |
| `assert` | `else` | `if` | `pass` | `yield` |
| `break` | `except` | `import` | `print` | |
| `class` | `exec` | `in` | `raise` | |
| `continue` | `finally` | `is` | `return` | |
| `def` | `for` | `lambda` | `try` | |

---

## 7. Modules & Classes (PEP 8 Supplement)

The Code Standard is written for standalone scripts. The conventions below extend it for **modules, packages, and classes** using PEP 8 where the Code Standard is silent. Remember: if anything here conflicts with Sections 1–6 above, the Code Standard rule wins.

### Module Organization

Modules (files intended to be imported, not run directly) follow the same ordering as Section 2, but replace "Main program code" with class and function definitions. There should be **no top-level executable code** other than definitions and constants.

Any test or demo code must be guarded so it only runs when the file is executed directly:

```python
if __name__ == "__main__":
    # Test or demo code only
    pass
```

### Class Naming

The Code Standard requires lowercase-initial `camelCase` for functions and variables. It does not address class names. Per PEP 8, **class names use `PascalCase`** (each word capitalized, no underscores):

```python
# Good
class UltrasonicSensor:
class LineFinder:
class ColorDetector:

# Bad
class ultrasonic_sensor:
class ultrasonicSensor:
```

Method and instance-variable names still follow the Code Standard's `camelCase` convention:

```python
class UltrasonicSensor:
    def getDistance(self):
        return self.lastReading  # camelCase per Code Standard
```

### Docstrings

The Code Standard requires comments explaining blocks of code (Section 5). For modules, classes, and public functions/methods, PEP 8 and [PEP 257](https://peps.python.org/pep-0257/) recommend **docstrings** — a triple-quoted string as the first statement of the definition:

```python
def computeDistance(duration):
    """
    Compute distance from an ultrasonic sensor pulse.

    duration -- pulse round-trip time in seconds
    Returns distance in centimeters.
    """
    return (duration * 34300) / 2  # cm
```

For classes:

```python
class LineFinder:
    """
    Interface for the line-following sensor array.

    Reads reflectance values and determines line position.
    """
```

Docstrings **complement** — but do not replace — inline `#` comments required by the Code Standard.

### Module-Level Variables

The Code Standard prohibits global variables (Section 4). At module scope, only **constants** (`ALL_CAPS_WITH_UNDERSCORES`) are acceptable:

```python
# Good — constants at module scope
MAX_DISTANCE = 400  # centimeters
DEFAULT_TIMEOUT = 0.03  # seconds

# Bad — mutable state at module scope
lastReading = 0
sensorReady = False
```

All mutable state belongs inside classes or functions.

### `__init__.py` Files

`__init__.py` files still require a **file header** (Section 1) if they contain any code beyond bare re-exports. If the file only re-exports names, a minimal header with the package description is sufficient:

```python
# Package: basehat
# File:    __init__.py
# Date:    18 February 2026
# ...
#
# Public interface for the basehat sensor package.

from .ultrasonicSensor import UltrasonicSensor
from .lineFinder import LineFinder
```

### Relative Imports

Within a package, use **relative imports** to reference sibling modules. This keeps the package self-contained:

```python
# Good — relative import within a package
from . import motors
from .color import ColorSensor

# Acceptable — absolute import
from buildhat.color import ColorSensor
```

The one-import-per-line rule (Section 6) still applies.
