import { Transform } from '../Common/Math';
import { Manifold } from './Collision';
import { PolygonShape } from './Shapes/PolygonShape';
export declare function CollidePolygons(manifold: Manifold, polyA: PolygonShape, xfA: Transform, polyB: PolygonShape, xfB: Transform): void;
