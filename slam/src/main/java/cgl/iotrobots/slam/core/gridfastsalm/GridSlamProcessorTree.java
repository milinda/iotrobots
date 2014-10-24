package cgl.iotrobots.slam.core.gridfastsalm;

import com.google.common.collect.Multimap;

import java.util.ArrayList;
import java.util.List;

public class GridSlamProcessorTree {
    List<TNode> getTrajectories() {
        List<TNode> v = new ArrayList<TNode>();
        Multimap<TNode, TNode> parentCache = new ;
        TNodeDeque border;

        for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
            TNode* node=it->node;
            while(node){
                node->flag=false;
                node=node->parent;
            }
        }

        for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
            TNode* newnode=new TNode(* (it->node) );

            v.push_back(newnode);
            assert(newnode->childs==0);
            if (newnode->parent){
                parentCache.insert(make_pair(newnode->parent, newnode));
                //cerr << __PRETTY_FUNCTION__ << ": node " << newnode->parent << " flag=" << newnode->parent->flag<< endl;
                if (! newnode->parent->flag){
                    //cerr << __PRETTY_FUNCTION__ << ": node " << newnode->parent << " flag=" << newnode->parent->flag<< endl;
                    newnode->parent->flag=true;
                    border.push_back(newnode->parent);
                }
            }
        }

        //cerr << __PRETTY_FUNCTION__ << ": border.size(INITIAL)=" << border.size() << endl;
        //cerr << __PRETTY_FUNCTION__ << ": parentCache.size()=" << parentCache.size() << endl;
        while (! border.empty()){
            //cerr << __PRETTY_FUNCTION__ << ": border.size(PREPROCESS)=" << border.size() << endl;
            //cerr << __PRETTY_FUNCTION__ << ": parentCache.size(PREPROCESS)=" << parentCache.size() << endl;
            const TNode* node=border.front();
            //cerr << __PRETTY_FUNCTION__ << ": node " << node << endl;
            border.pop_front();
            if (! node)
                continue;

            TNode* newnode=new TNode(*node);
            node->flag=false;

            //update the parent of all of the referring childs
            pair<TNodeMultimap::iterator, TNodeMultimap::iterator> p=parentCache.equal_range(node);
            double childs=0;
            for (TNodeMultimap::iterator it=p.first; it!=p.second; it++){
                assert(it->second->parent==it->first);
                (it->second)->parent=newnode;
                //cerr << "PS(" << it->first << ", "<< it->second << ")";
                childs++;
            }
            ////cerr << endl;
            parentCache.erase(p.first, p.second);
            //cerr << __PRETTY_FUNCTION__ << ": parentCache.size(POSTERASE)=" << parentCache.size() << endl;
            assert(childs==newnode->childs);

            //unmark the node
            if ( node->parent ){
                parentCache.insert(make_pair(node->parent, newnode));
                if(! node->parent->flag){
                    border.push_back(node->parent);
                    node->parent->flag=true;
                }
            }
            //insert the parent in the cache
        }
        //cerr << __PRETTY_FUNCTION__ << " : checking cloned trajectories" << endl;
        for (unsigned int i=0; i<v.size(); i++){
            TNode* node= v[i];
            while (node){
                //cerr <<".";
                node=node->parent;
            }
            //cerr << endl;
        }

        return v;

    }getTrajectories() const{
        TNodeVector v;
        TNodeMultimap parentCache;
        TNodeDeque border;

        for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
            TNode* node=it->node;
            while(node){
                node->flag=false;
                node=node->parent;
            }
        }

        for (ParticleVector::const_iterator it=m_particles.begin(); it!=m_particles.end(); it++){
            TNode* newnode=new TNode(* (it->node) );

            v.push_back(newnode);
            assert(newnode->childs==0);
            if (newnode->parent){
                parentCache.insert(make_pair(newnode->parent, newnode));
                //cerr << __PRETTY_FUNCTION__ << ": node " << newnode->parent << " flag=" << newnode->parent->flag<< endl;
                if (! newnode->parent->flag){
                    //cerr << __PRETTY_FUNCTION__ << ": node " << newnode->parent << " flag=" << newnode->parent->flag<< endl;
                    newnode->parent->flag=true;
                    border.push_back(newnode->parent);
                }
            }
        }

        //cerr << __PRETTY_FUNCTION__ << ": border.size(INITIAL)=" << border.size() << endl;
        //cerr << __PRETTY_FUNCTION__ << ": parentCache.size()=" << parentCache.size() << endl;
        while (! border.empty()){
            //cerr << __PRETTY_FUNCTION__ << ": border.size(PREPROCESS)=" << border.size() << endl;
            //cerr << __PRETTY_FUNCTION__ << ": parentCache.size(PREPROCESS)=" << parentCache.size() << endl;
            const TNode* node=border.front();
            //cerr << __PRETTY_FUNCTION__ << ": node " << node << endl;
            border.pop_front();
            if (! node)
                continue;

            TNode* newnode=new TNode(*node);
            node->flag=false;

            //update the parent of all of the referring childs
            pair<TNodeMultimap::iterator, TNodeMultimap::iterator> p=parentCache.equal_range(node);
            double childs=0;
            for (TNodeMultimap::iterator it=p.first; it!=p.second; it++){
                assert(it->second->parent==it->first);
                (it->second)->parent=newnode;
                //cerr << "PS(" << it->first << ", "<< it->second << ")";
                childs++;
            }
            ////cerr << endl;
            parentCache.erase(p.first, p.second);
            //cerr << __PRETTY_FUNCTION__ << ": parentCache.size(POSTERASE)=" << parentCache.size() << endl;
            assert(childs==newnode->childs);

            //unmark the node
            if ( node->parent ){
                parentCache.insert(make_pair(node->parent, newnode));
                if(! node->parent->flag){
                    border.push_back(node->parent);
                    node->parent->flag=true;
                }
            }
            //insert the parent in the cache
        }
        //cerr << __PRETTY_FUNCTION__ << " : checking cloned trajectories" << endl;
        for (unsigned int i=0; i<v.size(); i++){
            TNode* node= v[i];
            while (node){
                //cerr <<".";
                node=node->parent;
            }
            //cerr << endl;
        }

        return v;

    }
}
