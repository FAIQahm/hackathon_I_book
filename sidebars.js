// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Chapter 1: Physical AI & ROS 2',
      items: ['chapter-1/index'],
    },
    {
      type: 'category',
      label: 'Chapter 2: Simulation with Gazebo',
      items: ['chapter-2/index'],
    },
    {
      type: 'category',
      label: 'Chapter 3: Vision-Language-Action Models',
      items: ['chapter-3/index'],
    },
  ],
};

module.exports = sidebars;
