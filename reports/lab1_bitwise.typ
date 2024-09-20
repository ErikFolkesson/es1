#[
  #set align(center)
  = Lab 1 - DVA454 HT2024
  Johan Sandred - jsd21003 \
  Erik Folkesson - efn24005
]

== 1. How do you terminate the debugging session?
Either press the red terminate button in the toolbar or press Ctrl+F2.

== 2. How do you terminate the program running on the dev board?

Press the reset button on the board.

== 3. Operations

=== Bitwise operations
```
&  bitwise and
^  bitwise xor
|  bitwise or
~  bitwise not
<< bitwise left shift
>> bitwise right shift
```

=== Logical operations
```
&& logical and
|| logical or
```

===  Relational operators
```
== equal
!= not equal
<  less than
>  greater than
<= less or equal than
>= greater or equal than
```

== Questions
#set enum(numbering: "a)")

+ What is the value of the 'result' variable at each occasion it is assigned a value?

  #table(
  columns: (auto, auto),
  table.header(
    [*Section*], [*Value*]
  ),
  "and", "1",
  "and variant", "3",
  "or", "3",
  "xor", "2",
  "not", "254",
  "right shift", "1",
  "right shift again", "8",
  "left shift", "2",
  "left shift again", "128",
  "logical and", "true",
  "logical or", "true",
)

+ What happens at the end of the program.

The `result` variable has the value `TRUE` (1) and the execution enters an infinite loop and gets stuck there, which means the program never finishes its execution.
