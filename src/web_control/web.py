# TODO: Rewrite to english
"""
I need you to think through and write code for a monitoring system.

What is needed: a web server that can be added with a single library, so monitoring can grow into a web page. It should be enough to simply inherit from a monitoring base class, and the monitoring class should automatically extract from it:
	•	a dictionary with the content to display: variables, their values, and their access type (rw), with the ability to change values from the web interface
	•	a list of functions and arguments that can be called externally by providing the required arguments
	•	a server implementation class that receives changes and sends updated data over WebSocket so the page updates live

The page should support:
	•	displaying text
	•	displaying values with an optional ability to edit them
	•	a special window type for showing logs from a logger instance (also passed as an instance in the dictionary), displayed as a table with scrolling
	•	a large structured text list, for example from a list
	•	grouping into flexible groups where the interface adapts to the contents of each group
	•	one instance being able to return multiple groups, possibly with nested groups

Dictionaries should either be sent in full each time or not sent at all. Dictionaries should be stored in the class that reads data from them, and for example, where a value used to be, there may instead be a function that must be called to get the data. There should also be a way to force the class to refresh the structure.

So the idea is this: we have a static structure that the class sends once, and it returns methods from which new data can also arrive as a stream. If there are no updates, the method should return nothing. These will be iterators.

And mark up one class, for example SkyWatcher.
"""
