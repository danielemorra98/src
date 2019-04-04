# Coding style
The following rules are intended to simplify the life of contributors by making the code more readable and self-documenting.
Most of them are consistent with [PEP-8](https://www.python.org/dev/peps/pep-0008/).

## Table of Contents
- [Code layout](#code-layout)
  - [Spacing](#spacing)
  - [Line length](#line-length)
  - [Line break](#line-break)
  - [Blank lines](#blank-lines)
- [Whitespace](#whitespace)
- [Naming conventions](#naming-conventions)
  - [Packages](#packages)
  - [Modules](#modules)
  - [Classes](#classes)
  - [Functions](#functions)
  - [Constants](#constants)
  - [Variables](#variables)
- [Comments](#comments)
  - [Block comments](#block-comments)
  - [Inline comments](#inline-comments)
  - [Docstrings](#docstrings)

## Code layout

### Spacing
Always use 4 spaces per indentation level. Never ever use tabs.
*Hint: setup your editor to use spaces instead of tabs* 

### Line length
Lines should not be longer than 80 characters. It is ok to increase this limit to 100 when it improves the overall readability.

### Line break
The preferred way of wrapping long lines is by using Python's implied line continuation inside parentheses, brackets and braces. Long lines can be broken over multiple lines by wrapping expressions in parentheses. These should be used in preference to using a backslash for line continuation. Long mathematical expressions should be broken after a binary operator.
It is recommended to indent the second part of the expression to be aligned with the beginning of the first, but you are free to pick another style if it improves local readability. 

### Blank lines
Top-level functions and class definitions shall be surrounded with two blank lines.
Class should be preceded by a single blank line.
Other blank lines can be added sporadically to isolate blocks of related code.

## Whitespace

A single whitespace should be added:
- after a comma, except than right before a closing parenthesis
- before and after a binary operator
- before and after the assignment operator (except than in keyword args)

Whitespace should NOT be added:
- at the end of a line
- before a comma, colon, semicolon
- between function name and opening parenthesis
- to align variable declarations
- before colon in lambda functions

## Naming conventions

### Packages
Packages should have short, all-lowercase names. The use of underscores is discouraged.

### Modules 
Modules should have short, all-lowercase names. Underscores are recommended to separate words in the module name.

### Classes
Class names should use the UpperCamelCase convention.

### Functions
Function names should be lowercase, with words separated by underscores to improve readability.
Class methods are functions too, and should follow the very same convention.
Non-public methods should be prefixed by an underscore.

### Constants
Constants are written in all capital letters with underscores separating words. 

### Variables
Variable names should be lowercase, with words separated by underscores.

## Comments

### Block comments
Block comments apply to some code that follows them. They are indented to the same level as that code. Each line of a block comment starts with a # and a single space.

### Inline comments
Inline comments to clarify the meaning of variables during declaration, or to add a very short note about a piece of code.  These comments should be separated by at least two spaces from the statement. They should start with a # and a single space.

### Docstrings
Docstrings are meant to provide detailed info about the meaning of public modules, functions, classes, and methods. Docstrings are not necessary for non-public methods, but you should have a comment that describes what the method does. This comment should appear after the def line.
Docstrings of functions are recommended to include info about parameters and return values, according to the [Numpy specification](https://numpydoc.readthedocs.io/en/latest/format.html#docstring-standard).
