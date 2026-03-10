# TODO: Imlement
"""
Task:
Design and implement a Python monitoring framework that can be integrated into an existing application with minimal boilerplate. The framework must expose application state and actions through a live web UI.

Core goal

A developer should be able to add monitoring to any class by inheriting from a monitoring base class, for example SkyWatcherMonitorMixin, and describing its monitorable structure. The framework should automatically introspect and expose:
	1.	Variables / fields
Values to display in the UI, including:
	•	current value
	•	type / renderer
	•	access mode: read-only or read-write (ro / rw)
	•	optional setter callback for writable fields
	2.	Callable actions / commands
Functions that can be invoked from the UI, including:
	•	function name
	•	human-readable label
	•	argument schema
	•	argument types
	•	optional default values
	•	return value or status reporting
	3.	Streaming updates
The server should push updates over WebSocket so the web page updates live without polling.

⸻

Functional requirements

1. Monitoring integration model
The framework should be packaged as a reusable library.

A developer should be able to write something like:
	•	inherit from a monitoring base class or mixin
	•	describe monitoring groups and fields
	•	optionally expose methods as remote actions
	•	start the monitoring web server with one line or very little glue code

Example target usage:
	•	SkyWatcher class is annotated or structured so that monitoring is exposed automatically
	•	the monitoring layer should not require manually building frontend JSON by hand on every update

⸻

2. Static structure + dynamic data
The system must distinguish between:

A. Static structure
Sent once on connection, or when explicitly refreshed. This includes:
	•	page/group layout
	•	field definitions
	•	action definitions
	•	renderer types
	•	nesting of groups
	•	metadata required by the frontend to build the UI

B. Dynamic updates
Sent incrementally over WebSocket only when data changes.
If there are no updates, nothing should be sent.

This means the monitored class should expose:
	•	a method to build or return the static structure
	•	one or more iterators / generators / async iterators that yield updates
	•	an optional “force refresh structure” mechanism

⸻

3. Dictionary handling
For structured data dictionaries:
	•	either send the full dictionary snapshot
	•	or send nothing if nothing changed
	•	do not partially diff arbitrary nested dictionaries unless explicitly designed for that

The monitoring framework should internally own the last known dictionary state for comparison.

Dictionary entries may contain:
	•	plain values
	•	descriptors / metadata objects
	•	callables that should be invoked to obtain the current value

So the framework must support deferred evaluation, where a field definition may point to a function instead of storing the raw value directly.

⸻

4. UI capabilities
The web page must support at least these UI block types:
	1.	Plain text
	•	static or dynamic text blocks
	2.	Value fields
	•	show current value
	•	optionally editable if rw
	•	edits from UI should call the appropriate setter or mutation handler
	3.	Action calls
	•	render callable functions as buttons/forms
	•	arguments should be entered in the UI
	•	action execution result should be displayed
	4.	Logger view
	•	special renderer for a logger/log stream instance
	•	display logs in a scrollable table or log panel
	•	new logs should stream live
	5.	Structured text list
	•	render large structured textual output from lists or similar sources
	6.	Flexible groups
	•	UI layout should adapt to group contents
	•	groups may contain mixed item types
	•	groups may be nested
	•	one monitored instance may expose multiple top-level groups

⸻

5. Server architecture
Implement a server layer that:
	•	hosts the web UI
	•	exposes a WebSocket endpoint for live updates
	•	accepts user edits and action invocations from the frontend
	•	dispatches them to the appropriate monitored instance
	•	sends update events back to connected clients

The architecture should be cleanly split into:
	•	monitoring model / descriptors
	•	monitored object adapter / introspection layer
	•	WebSocket event transport
	•	web server
	•	frontend rendering layer

⸻

6. Extensibility
The design should make it easy to add:
	•	new field/render types
	•	new container/group types
	•	custom serializers
	•	custom action argument types
	•	multiple monitored instances on one page
	•	nested monitors or composite monitors

⸻

7. Concurrency / update model
Choose a concrete runtime model and explain it clearly:
	•	synchronous iterators, or
	•	async iterators, or
	•	background tasks with queues

Use one coherent approach throughout the implementation.

The system should safely support:
	•	multiple clients
	•	live updates
	•	user-initiated writes
	•	action execution
	•	log streaming

⸻

8. Error handling
Define behavior for:
	•	invalid field edits
	•	invalid action arguments
	•	exceptions inside monitored methods
	•	disconnected WebSocket clients
	•	stale structure vs refreshed structure

The UI should receive structured error responses.

⸻

Deliverables

Provide:
	1.	Architecture explanation
	•	major components
	•	responsibilities
	•	data flow
	•	lifecycle of structure vs updates
	2.	Concrete Python implementation
	•	minimal but working prototype
	•	clear separation of concerns
	•	type annotations
	3.	Web server implementation
	•	recommended stack is acceptable if justified
	•	include WebSocket support
	4.	Minimal frontend
	•	enough HTML/JS to render the structure
	•	subscribe to updates
	•	allow editing fields and invoking actions
	5.	Example monitored class
	•	use SkyWatcher as the example
	•	include:
	•	plain text
	•	rw and ro fields
	•	actions with arguments
	•	nested groups
	•	logger panel
	•	structured list output
	•	dynamic update stream
	6.	Example of force-refreshing structure
	7.	Short discussion of tradeoffs
	•	why this design
	•	what is simplified
	•	what would need to change for production use

⸻

Constraints
	•	Minimize boilerplate for application developers
	•	Avoid requiring manual frontend wiring for every field
	•	Prefer explicit, typed descriptors over fragile magic where possible
	•	Keep the prototype compact, understandable, and extensible
	•	The result should be runnable as a starting point, not just pseudocode

⸻

Output format

Structure the answer as:
	1.	Problem framing
	2.	Design decisions
	3.	Data model
	4.	Server and WebSocket flow
	5.	Python code
	6.	Frontend code
	7.	SkyWatcher example
	8.	Limitations and production improvements

"""
