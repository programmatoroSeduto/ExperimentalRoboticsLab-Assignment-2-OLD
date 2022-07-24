

The RoboCluedo Project Documentaton
====================================

.. toctree::
   :titlesonly:
   :caption: README and other docs
   :glob:
   
   project readme <README.md>
   reStructured directives <./_docs/rest-directives.md>
   _docs/*


Packages documentation
------------------------

.. toctree::
	:titlesonly:
	:caption: project packages
	
	./robocluedo_armor_interface/index.rst
	./robocluedo_movement_controller/index.rst
	./robocluedo_mission_manager/index.rst
	./robocluedo_rosplan_interface/index.rst
	

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
	
	@startuml

	title Packages - Component Diagram

	package "Front End" {
		component [Graphic User\nInterface] as GUI
	}

	cloud Internet {
	}
	 
	node "Middle Tier" {
		[Business Logic]
		[Data Access] as DA  
		interface IMath as Math
		interface "IItems" as Items
	} 

	database "PostgreSQL\n" {
		[Stored Procs]
	}

	GUI -down-> Internet
	Internet -down-( Math
	[Business Logic] -up- Math
	DA -- Items
	[Business Logic] --( Items
	DA .. [Stored Procs]

	@enduml
