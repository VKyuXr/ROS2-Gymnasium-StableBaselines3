#set page(paper: "jis-b5")
#set text(font: ("STSong", "Times New Roman"))
#set page(numbering: "1 / 1") 
#show raw.where(block: false): box.with( 
  fill: luma(233), 
  inset: (x: 2pt, y: 0pt), 
  outset: (y: 3pt), 
  radius: 2pt, 
) 
#show raw.where(block: true): block.with( 
  fill: luma(233), 
  inset: 10pt, 
  radius: 5pt, 
  width: 100%,
)
#set heading(numbering: "1.")
#show heading.where(): set text(font: ("Times New Roman", "Microsoft YaHei"))
#show heading.where(level: 1): set align(center)
#show heading.where(level: 1): set text(size: 20pt)
#show table.where(): set align(center)
#show table.cell.where(): set align(left)
#show heading.where(level: 1): set align(center)

#align(center)[#text("ROS2学习笔记", font: ("Times New Roman", "STSong"), size: 30pt)]
#align(center)[#text("——从入门到入土", size: 16pt)]
#align(right)[Created with Typst]
#align(center)[vkyuxr, Qwen]

#table(
  columns: 1, [
ROS2（Robot Operating System 2）是一个开源的机器人操作系统，旨在提供一个灵活的框架来开发机器人软件。
它支持多种编程语言和平台，具有分布式计算、实时性能和安全性等特点。

本文档的内容来源于我学习ROS2过程中的笔记，其来源于学习过程中的各个小工程。
本文档的知识框架可能并不全面，但是如果只是想让机器人动起来的话，应该是够用了。
])


#outline()
#pagebreak()

= ROS2原理说明
#line(length: 100%) 

#include "BK-ROS2-1.typ"
#pagebreak()

= ROS2项目结构
#line(length: 100%) 

#include "BK-ROS2-2.typ"
#pagebreak()

= joint的控制
#line(length: 100%) 

#include "BK-ROS2-3.typ"
#pagebreak()

= sensor的使用
#line(length: 100%) 

#include "BK-ROS2-4.typ"
#pagebreak()

= 使用ROS2进行强化学习
#line(length: 100%) 

#include "BK-ROS2-5.typ"
#pagebreak()

#align(top)[
  #v(20pt) \
  #text("ROS2", font: ("Times New Roman"), size: 80pt) \
  #text(
    [学 \ 习 \ 笔 \ 记],
    font: ("STSong"),
    size: 50pt,
    fill: luma(200)
  )
]

#align(bottom)[
  #align(right)[Version 0.3.1]
  #line(length: 100%) 
  #align(right)[#datetime.today().display("[year]年[month]月[day]日")]
  #align(right)[vkyuxr, Qwen]
  #v(150pt)
]

