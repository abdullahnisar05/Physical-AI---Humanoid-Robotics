import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manual sidebar structure for the Physical AI & Humanoid Robotics book
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: [
        'introduction/01-welcome',
        'introduction/02-foundations',
        'introduction/03-learning-outcomes',
        'introduction/04-sensor-overview',
        'introduction/05-why-physical-ai-matters',
        'introduction/06-hardware-requirements',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/01-introduction',
        'module-1/02-installation',
        'module-1/03-core-concepts',
        'module-1/04-pub-sub',
        'module-1/05-services-actions',
        'module-1/06-parameters-packages',
        'module-1/07-urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/01-introduction',
        'module-2/02-setup',
        'module-2/03-formats',
        'module-2/04-physics',
        'module-2/05-sensors',
        'module-2/06-unity',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3/01-overview',
        'module-3/02-setup',
        'module-3/03-assets',
        'module-3/04-synthetic-data',
        'module-3/05-isaac-ros',
        'module-3/06-navigation',
        'module-3/07-sim-to-real',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/01-introduction',
        'module-4/02-voice-to-text',
        'module-4/03-cognitive-planning',
        'module-4/04-language-to-action',
        'module-4/05-multi-modal',
        'module-4/06-capstone',
      ],
    },
    {
      type: 'category',
      label: 'Assessments & Projects',
      items: [
        'assessments/01-overview',
        'assessments/02-capstone-evaluation',
        'assessments/03-exercises',
        'assessments/04-quizzes',
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'appendices/01-glossary',
        'appendices/02-references',
        'appendices/03-resources',
        'appendices/04-troubleshooting',
        'appendices/05-capstone-project',
        'appendices/06-ros2-reference',
        'appendices/07-isaac-resources',
        'appendices/08-next-steps',
        'appendices/09-additional-resources',
        'appendices/10-conclusion',
        'appendices/11-homework-assignments',
        'appendices/12-curriculum-evolution',
        'appendices/13-advanced-glossary',
        'appendices/tasks',
        'appendices/plan',
      ],
    },
  ],
};

export default sidebars;
