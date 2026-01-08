/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Welcome',
    },
    {
      type: 'category',
      label: 'Chapters',
      items: [
        'chapter-1/index',
        'chapter-2/index',
        'chapter-3/index',
      ],
    },
  ],
};

module.exports = sidebars;
