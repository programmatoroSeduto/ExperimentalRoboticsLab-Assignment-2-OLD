

The RoboCluedo Project Documentaton
====================================

.. toctree::
   :titlesonly:
   :caption: README and other docs
   
   project readme <README.md>
   reStructured directives <./_docs/rest-directives.md>


Packages documentation
------------------------

.. toctree::
	:titlesonly:
	:caption: project packages
	
	pkg cluedo_armor_interface <./robocluedo_armor_interface/index.rst>
	

Indices and tables
-------------------

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


test graphviz
--------------

.. graphviz::

   digraph {
      "From" -> "in the middle" -> "To";
      "1 to" -> "to 2";
   }

test UML
----------

.. uml::

   Alice -> Bob: Hi!
   Alice <- Bob: How are you?
