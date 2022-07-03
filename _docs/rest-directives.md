
# ReStructured Directives -- practical documentation

## Include a file into another file

:warning: NOt working for .md files! :warning:

```rst
.. include:: <yourfile.format>
```

## TocTree directive

:point_up: a reST file can contain more than one `.. toctree::` directive. 

general syntax:

```rst
.. toctree::
	...
	:option: value
	...
	
	...
	file.name
	...
```

file name syntax:

- `path/of/the/file.tp` prints the title of the page and the link
- `this page <path/of/the/file.tp>` print *this page* with the link to the page

## Index Template

:point_up: reSt is truly horrible. Use the reST only for indexing the documentation. And nothing else. 

```rst
.. _your-tag:

page title
==========

.. toctree::
	:maxdepth: 1
	:caption: ToC title
	:titlesonly:
	:glob:
	:hidden:
	
	...
	link text <./path/link.format>
	...

```

the page can be referred using both `:ref:<your-tag>` and `:any:<your-tag>`. 

## Add some code

### from Python

This directive inserts the entire documentation related to the py file you're referring to. Just one instructor for generating the documentation. 

:point_up: in Sphinx, you say where to generate the documentation, pointing the page with this directive. In Doxygen instead, the file system is fixed. 

```rst
.. automodule:: your_py_file
    :members:
    :noindex:
```

### from Doxygen/Breathe

same observations as before for what concerns the cpp files. 

```rst
.. doxygenfile:: your-cpp-file.cpp
    :project: your-project-name
```
