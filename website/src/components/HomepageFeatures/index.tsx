import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Complete Curriculum',
    description: (
      <>
        Comprehensive learning path covering ROS 2, simulation, AI integration,
        and Vision-Language-Action systems for humanoid robotics.
      </>
    ),
  },
  {
    title: 'Hands-On Learning',
    description: (
      <>
        Practical exercises and capstone projects that build real-world
        skills in Physical AI and humanoid robotics development.
      </>
    ),
  },
  {
    title: 'Cutting-Edge Tech',
    description: (
      <>
        Learn with the latest tools: NVIDIA Isaac, Gazebo, Unity,
        and state-of-the-art VLA (Vision-Language-Action) systems.
      </>
    ),
  },
];

function Feature({title, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
