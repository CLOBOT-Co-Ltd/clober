Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~

Contributors must sign-off each commit by adding a `Signed-off-by: ...`
line to commit messages to certify that they have the right to submit
the code they are contributing to the project according to the
[Developer Certificate of Origin (DCO)](https://developercertificate.org/).

Contributing to clobot
============================================

**Thanks for your attention to clobot’s robot and open-source packages.**

**For more details about clober and RMF(Robotics Middleware Framework), read this documentation.**

* [Clober Manual](https://app.gitbook.com/@clobot-git/s/clober-manual/)

**If you want to suggest an idea or make a better code for the robot, just sending your pull requests. But, please comply following rules.**


**License**
--------------
By contributing, you agree that your contribution will be licensed under its Apache License.

### Apach 2 License

https://www.apache.org/licenses/LICENSE-2.0


**Coding Style**
--------------

We adapt coding convention recommended from ROS1/ROS2, as follow as


  * http://wiki.ros.org/StyleGuide
  * https://docs.ros.org/en/foxy/Contributing/Code-Style-Language-Versions.html

If you want to contribute repositories forked from open-rmf repositories, please follow open-rmf
 coding convention. See this link

 * https://openrmf.readthedocs.io/en/latest/contributing/index.html#c


**Commit/PR Message**
--------------

### **Message template**

```
type: title

 - write body 1
 - write body 2

close/fix/resolve #number_of_issue
related to #number_of_issue
```
You can configure the message template as following command
`git config --global commit.template /PATH/YOUR/.gitmessage`

### **Rule**

* first of all, put 'type' of the commit message

 |Type    |Description|
 |:---    |:----------|
 |feat    |New feature|
 |fix     |bug fix|
 |docs    |changes to documentation|
 |style   |formatting, missing semi colons, etc / no code change|
 |refactor|refactoring production code|
 |chore   |updating grunt tasks etc / no production code change|

* And remember
  * Capitalize the subject line
  * Use the imperative mood in the subject line
  * Do not end the subject line with a period
  * Separate subject from body with a blank line
  * Use the body to explain what and why vs. how
  * Can use multiple lines with "-" for bullet points in body

**Contact**
--------------

Please contact on email : ros@clobot.co.kr
