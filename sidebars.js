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
        'chapter-1/chapter-1',
        'chapter-2/chapter-2',
        'chapter-3/chapter-3',
      ],
    },
  ],
};

module.exports = sidebars;
